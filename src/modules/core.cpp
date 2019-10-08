#include "core.h"

// private declarations for core

namespace core
{

	// private members

	bool initialized = false;
	ctrl_state_t state;

	float scaling_factor = 0.10f;

	gazebo::transport::NodePtr node;
	gazebo::transport::SubscriberPtr sub_lidar;
	gazebo::transport::SubscriberPtr sub_camera;
	gazebo::transport::SubscriberPtr sub_pose;
	gazebo::transport::PublisherPtr pub_velcmd;
	gazebo::transport::PublisherPtr pub_world;
	gazebo::msgs::WorldControl ctrl_msg;

	lidar_t  lidar_data;
	camera_t camera_data;
	pose_t   pose_data;
	vel_t    vel_data;
	pos_t    goal = { 3.f, 3.f, 0.f };
	obs_t    nearest_obs;

	// private methods

	void
	make_debug_data();

	void
	align_windows();
	
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
	flctr_goal_nav(pos_t& goal);

	void
	flctrl_obs_avoid();

}

// --------------------------------------------------------------------------------
// private method defintions for ::core
// --------------------------------------------------------------------------------

inline void
core::make_debug_data()
{
	constexpr auto WIDTH = 20;

	debug::dout << std::left      
		<< std::setw(WIDTH) << "State:"             << std::left << core::ctrl_state_names[core::state] << "\n"
		<< std::setw(WIDTH) << "Position:"          << std::left << core::pose_data.pos << "\n"
		<< std::setw(WIDTH) << "Orientation:"       << std::left << core::pose_data.pos << "\n"
		<< std::setw(WIDTH) << "Velocity:"          << std::left << core::vel_data << "\n"
		<< "\n"
		<< std::setw(WIDTH) << "Goal:"              << std::left << core::goal << "\n"
		<< std::setw(WIDTH) << "Nearest obstacle:"  << std::left << core::nearest_obs << "\n"
		<< std::flush;
}

void
core::align_windows()
{
	// needs newer version of OpenCV to implement this method dynamically
	size_t i = 0, total_x = 0;
	for (const auto& wndw : WNDW_HANDLES)
	{
		cv::moveWindow(wndw, WNDW_ORIGIN[0] + total_x, WNDW_ORIGIN[1] );
		total_x += (WNDW_WIDTHS[i++] + WNDW_MARGIN);
	}
}

void
core::callback_lidar(ConstLaserScanStampedPtr& msg)
{	
	// populate lidar data
	// guarded by mutex internally
	lidar_data.set(msg);
}

void
core::callback_camera(ConstImageStampedPtr& msg)
{
	camera_data.mutex.lock();

	camera_data.img_width  = msg->image().width();
	camera_data.img_height = msg->image().height();
	camera_data.data       = msg->image().data().c_str();

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

			pose_data.pos.x = pose.position().x();
			pose_data.pos.y = pose.position().y();
			pose_data.pos.z = pose.position().z();

			pose_data.orient.w = pose.orientation().w();
			pose_data.orient.x = pose.orientation().x();
			pose_data.orient.y = pose.orientation().y();
			pose_data.orient.z = pose.orientation().z();

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
	core::node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	core::node->Init();

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

	// set initial state
	core::state = goal_nav;

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
		cv::imshow(WNDW_LIDAR, lidar_data.get_img());
		
		// show camera output
		cv::imshow(WNDW_CAMERA, camera_data.get_img());

		// run fuzzy logic controller
		core::flctrl();

		// publish velocity command
		core::publish_velcmd();

		debug::cout << "some more information\n";

		// show debug information
		debug::show(&core::make_debug_data);

		// align windows
		core::align_windows();
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
	using namespace core;

	// extract nearest obstacle
	nearest_obs = lidar_data.get_nearest_obs(pose_data.pos);

	// check distace no nearest obstacle
	// select appropriate fuzzy controller
	if (nearest_obs.dist < MAX_DIST_TO_OBSTACLE)
	{
		state = obs_avoid;
		//flctrl_obs_avoid();
	}
	else
	{
		state = goal_nav;
		flctr_goal_nav(goal);
	}
}

void
core::flctrl_obs_avoid()
{
	// load engine and check for readiness
	static fl::Engine* engine = fl::FllImporter().fromFile(PATH_FUZZY_OBS_AVOID);

	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);

	// variables (loaded once)
	static fl::InputVariable*  obs_dir    = engine->getInputVariable("obs_dir");
	static fl::InputVariable*  obs_dist   = engine->getInputVariable("obs_dist");
	static fl::OutputVariable* rob_vel    = engine->getOutputVariable("rob_veldir");
	static fl::OutputVariable* rob_angvel = engine->getOutputVariable("rob_velrot");

	// apply inputs
	obs_dir->setValue(core::nearest_obs.dir);
	obs_dist->setValue(core::nearest_obs.dist);

	// process data
	engine->process();

	// export outputs
	core::vel_data.trans = (float)rob_vel->getValue();
	core::vel_data.ang   = (float)rob_angvel->getValue();
}

void
core::flctr_goal_nav(pos_t& goal)
{
	// load engine and check for readiness
	static fl::Engine* engine = fl::FllImporter().fromFile(PATH_FUZZY_SIMPLE_NAVIGATOR);

	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);
	
	static fl::InputVariable* goal_dist    = engine->getInputVariable("goal_dist");
	static fl::InputVariable* goal_dir 	   = engine->getInputVariable("goal_dir");
	static fl::OutputVariable* robot_dir   = engine->getOutputVariable("robot_dir");
	static fl::OutputVariable* robot_speed = engine->getOutputVariable("robot_speed");	

	// get inputs
	float dir = pose_data.dir(goal);
	float dist = pose_data.dist(goal);

	// feed input to the fl engine
	goal_dir->setValue(dir);
	goal_dist->setValue(dist);
	
	// process inputs
	engine->process();
	
	// extract outputs
	vel_data.trans = core::scaling_factor * (float)robot_speed->getValue();
	vel_data.ang   = core::scaling_factor * (float)robot_dir->getValue();
}