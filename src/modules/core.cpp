#include "core.h"

// private declarations for core

namespace core
{

	// private members

	bool initialized = false;

	std::mutex cv_mutex;

	gazebo::transport::SubscriberPtr sub_lidar;
	gazebo::transport::SubscriberPtr sub_camera;
	gazebo::transport::SubscriberPtr sub_pose;
	gazebo::transport::PublisherPtr pub_velcmd;
	gazebo::transport::PublisherPtr pub_world;
	gazebo::msgs::WorldControl ctrl_msg;

	fl::Engine* fl_engine;

	lidar_t lidar_data;
	camera_t camera_data;
	pose_t pose_data;
	vel_t vel_data;

	ctrl_state_t state = ctrl_state_t::simple_nav;

	// private methods
	
	void
	callback_lidar(ConstLaserScanStampedPtr& msg);

	void
	callback_camera(ConstImageStampedPtr& msg);

	void
	callback_pose(ConstPosesStampedPtr& msg);

	void
	publish_velcmd();

	void
	flctrl();

	void 
	flctr_goal_navigator(core::pose_t goal);

	void
	flctrl_obs_avoid();

}

// --------------------------------------------------------------------------------
// private method defintions for ::core
// --------------------------------------------------------------------------------

void
core::callback_lidar(ConstLaserScanStampedPtr& msg)
{
	lidar_data.mutex.lock();

	auto& scan = msg->scan();

	lidar_data.scan = scan;
	lidar_data.angle_min = float(scan.angle_min());
	lidar_data.angle_max = scan.angle_max();
	lidar_data.angle_increment = float(scan.angle_step());
	lidar_data.range_min = float(scan.range_min());
	lidar_data.range_max = float(scan.range_max());

	assert(lidar_data.nranges == lidar_data.nintensities);

	lidar_data.sec = msg->time().sec();
	lidar_data.nsec = msg->time().nsec();
	lidar_data.nranges = scan.ranges_size();
	lidar_data.nintensities = scan.intensities_size();

	lidar_data.mutex.unlock();
}

void
core::callback_camera(ConstImageStampedPtr& msg)
{
	camera_data.mutex.lock();

	camera_data.width  = msg->image().width();
	camera_data.height = msg->image().height();
	camera_data.data   = msg->image().data().c_str();

	camera_data.mutex.unlock();
}

void
core::callback_pose(ConstPosesStampedPtr& msg)
{
	for (int i = 0; i < msg->pose_size(); i++)
	{
		auto& pose = msg->pose(i);

		if (pose.name() == "pioneer2dx")
		{
			pose_data.mutex.lock();

			pose_data.position.x = pose.position().x();
			pose_data.position.y = pose.position().y();
			pose_data.position.z = pose.position().z();

			pose_data.orientation.w = pose.orientation().w();
			pose_data.orientation.x = pose.orientation().x();
			pose_data.orientation.y = pose.orientation().y();
			pose_data.orientation.z = pose.orientation().z();

			pose_data.mutex.unlock();
		}
	}
}

// --------------------------------------------------------------------------------
// public method defintions for ::core
// --------------------------------------------------------------------------------

void
core::init(int argc, char** argv)
{

	// assert that system is not already initialized
	if (core::initialized)
		throw std::runtime_error("System is already initialized.");

	// load gazebo
	gazebo::client::setup(argc, argv);

	// create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// subsribe to gazebo topics (assign callbacks)
	core::sub_lidar  = node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", core::callback_lidar);
	core::sub_pose   = node->Subscribe("~/pose/info", core::callback_pose);
	core::sub_camera = node->Subscribe("~/pioneer2dx/camera/link/camera/image", core::callback_camera);

	// setup gazebo publishers
	core::pub_velcmd = node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");
	core::pub_world  = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

	//
	core::ctrl_msg.mutable_reset()->set_all(true);

	// publish control message
	core::pub_world->WaitForConnection();
	core::pub_world->Publish(core::ctrl_msg);

	// load fuzzy engine
	core::fl_engine = fl::FllImporter().fromFile(PATH_FUZZY_OBS_AVOID);

	// set initialization status
	core::initialized = true;
}

void
core::run()
{
	
	// assert that system is initialized
	if (not core::initialized)
		throw std::runtime_error("System is not initialized.");

	// loop
	while (true)
	{
		// sleep
		std::this_thread::sleep_for(RUN_FREQ_MS);

		// acquire key input from opencv and draw any imshow() windows
		// close app if ESC key has been pressed
		if (cv::waitKey(1) == 27) break;

		// show lidar outout
		cv::imshow(WNDW_LIDAR, lidar_data.get_img_safe());
		
		// show camera output
		cv::imshow(WNDW_CAMERA, camera_data.get_img_safe());		

		// run fuzzy lite controller
		core::flctrl();

		// publish velocity command
		core::publish_velcmd();
	}

	// shutdown gazebo
	gazebo::client::shutdown();
}

void
core::publish_velcmd()
{
	// convert pose to message; using the global velocity info
	static gazebo::msgs::Pose msg;
	gazebo::msgs::Set(&msg, vel_data.pose());

	// publish the velocity command
	core::pub_velcmd->Publish(msg);
}

void
core::flctrl()
{
	// process data
	lidar_data.mutex.lock();

	// extract dist and dir of nearest obstacle
	;

	std::cout << lidar_data.nranges << std::endl;

	lidar_data.mutex.unlock();

	// select appropriate fuzzy controller
	//if (core::state == simple_nav) flctrl_simple_nav();
	//if (core::state == obs_avoid)  flctrl_obs_avoid();
}

void
core::flctrl_obs_avoid()
{
	// load engine
	// core::fl_engine = fl::FllImporter().fromFile(PATH_FUZZY_OBS_AVOID);

	// check enginge
	if (std::string status; not fl_engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);

	// variables (loaded once)
	static fl::InputVariable*  obs_dir    = fl_engine->getInputVariable("obs_dir");
	static fl::InputVariable*  obs_dist   = fl_engine->getInputVariable("obs_dist");
	static fl::OutputVariable* rob_veldir = fl_engine->getOutputVariable("rob_veldir");
	static fl::OutputVariable* rob_velrot = fl_engine->getOutputVariable("rob_velrot");

	// apply inputs
	obs_dir->setValue(10.f);
	obs_dist->setValue(10.f);

	// process data
	fl_engine->process();

	// export outputs
	core::vel_data.dir = rob_veldir->getValue();
	core::vel_data.rot = rob_velrot->getValue();
}



void flctr_goal_navigator(core::pose_t goal)
{
	// load engine and check for readiness

	static fl::Engine* engine;
	engine = fl::FllImporter().fromFile(PATH_FUZZY_SIMPLE_NAVIGATOR);
	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);
	
	static fl::InputVariable* goal_dist   = engine.getInputVariable("goal_dist");
	static fl::InputVariable* goal_dir 	  = engine.getInputVariable("goal_dir");
	static fl::InputVariable* robot_dir   = engine.getInputVariable("robot_dir");
	static fl::InputVariable* robot_speed = engine.getInputVariable("robot_speed");

	//get inputs:





}