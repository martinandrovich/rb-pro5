#include "core.h"

// private declarations for core

namespace core
{

	// private members

	bool initialized = false;
	ctrl_state_t state;

	gazebo::transport::NodePtr node;
	gazebo::transport::SubscriberPtr sub_lidar;
	gazebo::transport::SubscriberPtr sub_camera;
	gazebo::transport::SubscriberPtr sub_pose;
	gazebo::transport::PublisherPtr pub_velcmd;
	gazebo::transport::PublisherPtr pub_world;
	gazebo::msgs::WorldControl ctrl_msg;

	lidar_t    lidar_data;
	camera_t   camera_data;
	pose_t     pose_data;
	vel_t      vel_data;
	pos_t      goal;
	obs_list_t nearest_obs;

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
	controller();

	void
	flctrl_obs_avoid(obs_list_t& obs_list, vel_t& vel_cmd);

	void
	flctrl_obs_avoid_OLD();

	void 
	flctr_goal_nav(pos_t& goal);

	void
	stop_vehicle();
	
	void
	_test_orientation();

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
		<< std::setw(WIDTH) << "Orientation:"       << std::left << core::pose_data.orient << "\n"
		<< std::setw(WIDTH) << "Velocity:"          << std::left << core::vel_data << "\n"
		<< "\n"
		<< std::setw(WIDTH) << "Goal:"              << std::left << core::goal << "\n"
		<< std::setw(WIDTH) << "Nearest obstacle:"  << std::left << nearest_obs["any"] << "\n"
		<< "";
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

	// set goal
	core::goal = { 5.f, 0.f, 0.f };

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

		// run main controller
		core::controller();

		// publish velocity command
		core::publish_velcmd();

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
core::controller()
{
	// extract nearest obstacles
	nearest_obs["center"] = lidar_data.get_nearest_obs(pose_data.pos, { 0.38, -0.38 });
	nearest_obs["right"]  = lidar_data.get_nearest_obs(pose_data.pos, {-1.37, -1.76 });
	nearest_obs["left"]   = lidar_data.get_nearest_obs(pose_data.pos, { 1.76,  1.37 });
	nearest_obs["any"]    = lidar_data.get_nearest_obs(pose_data.pos);

	// check distace no nearest obstacle
	// select appropriate fuzzy controller
	if (nearest_obs["any"].dist < MAX_DIST_TO_OBSTACLE)
	{
		state = obs_avoid;
		flctrl_obs_avoid(nearest_obs, vel_data);
	}
	else
	{
		state = goal_nav;
		_test_orientation();
		/*
		//Stop fuzzy navigation if goal is "reached" 
		if(pose_data.dist(goal) > 0.05) flctr_goal_nav(goal);
		else
		{
			stop_vehicle();
		}
		*/
		
	}
}

void
core::flctrl_obs_avoid(obs_list_t& obs_list, vel_t& vel_cmd)
{
	// load engine and check for readiness
	static fl::Engine* engine = fl::FllImporter().fromFile(PATH_FUZZY_OBS_AVOID);

	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);

	// variables (loaded once)
	static fl::InputVariable*  obs_c_dist = engine->getInputVariable("forward");
	static fl::InputVariable*  obs_r_dist = engine->getInputVariable("right");
	static fl::InputVariable*  obs_l_dist = engine->getInputVariable("left");
	static fl::OutputVariable* rob_vel    = engine->getOutputVariable("velocity");
	static fl::OutputVariable* rob_angvel = engine->getOutputVariable("omega");

	// apply inputs
	obs_c_dist->setValue(obs_list["center"].dist);
	obs_r_dist->setValue(obs_list["right"].dist);
	obs_l_dist->setValue(obs_list["left"].dist);

	// process data
	engine->process();

	// export outputs
	vel_cmd.trans = FUZZY_SCALING_FACTOR * (float)rob_vel->getValue();
	vel_cmd.ang   = FUZZY_SCALING_FACTOR * (float)rob_angvel->getValue();
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
	vel_data.trans = FUZZY_SCALING_FACTOR * (float)robot_speed->getValue();
	vel_data.ang   = FUZZY_SCALING_FACTOR * (float)robot_dir->getValue();
}

void
core::flctrl_obs_avoid_OLD()
{
	// load engine and check for readiness
	static fl::Engine* engine = fl::FllImporter().fromFile(PATH_FUZZY_OBS_AVOID);

	if (std::string status; not engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);

	// variables (loaded once)
	static fl::InputVariable*  obs_dir    = engine->getInputVariable("obs_dir");
	static fl::InputVariable*  obs_dist   = engine->getInputVariable("obs_dist");
	static fl::OutputVariable* rob_vel    = engine->getOutputVariable("rob_vel");
	static fl::OutputVariable* rob_angvel = engine->getOutputVariable("rob_angvel");

	// apply inputs
	obs_dir->setValue(nearest_obs["any"].dir);
	obs_dist->setValue(nearest_obs["any"].dist);

	// process data
	engine->process();

	// export outputs
	core::vel_data.trans = FUZZY_SCALING_FACTOR * (float)rob_vel->getValue();
	core::vel_data.ang   = FUZZY_SCALING_FACTOR * (float)rob_angvel->getValue();
}

void
core::stop_vehicle()
{
	core::vel_data.trans = 0.f;
	core::vel_data.ang   = 0.f;
}


void
core::_test_orientation()
{
	debug::cout << "Testing orientation...\n";
	static float i = 0;	
	i = 0.1;	
	core::vel_data.trans = 0.f;
	core::vel_data.ang = i;		
}