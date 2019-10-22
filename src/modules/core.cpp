#include "core.h"

// private declarations for core

namespace core
{

	// enumerations
	
	enum ctrl_state_t
	{
		goal_nav,
		obs_avoid
	};

	constexpr std::string_view ctrl_state_names[] =
	{
		[goal_nav]   = "goal navigator",
		[obs_avoid]  = "obstacle avoidance"
	};

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
	test_orientation();

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
	stop_vehicle();
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
		<< std::setw(WIDTH) << "Direction:"         << std::left << core::pose_data.orient.yaw << "\n"
		<< std::setw(WIDTH) << "Velocity:"          << std::left << core::vel_data << "\n"
		<< "\n"
		<< std::setw(WIDTH) << "Goal:"              << std::left << core::goal << "\n"
		<< std::setw(WIDTH) << "Nearest obstacle:"  << std::left << nearest_obs["any"] << "\n"
		<< "";
}

void
core::test_orientation()
{
	debug::cout << "Testing orientation...\n";
	core::vel_data.trans = 0.f;
	core::vel_data.ang = 0.1f;
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
	// populate camera data
	// guarded by mutex internally
	camera_data.set(msg);
}

void
core::callback_pose(ConstPosesStampedPtr& msg)
{
	for (int i = 0; i < msg->pose_size(); i++)
	{
		auto& pose = msg->pose(i);	

		// populate pose data
		// guarded by mutex internally

		if (pose.name() == "pioneer2dx")
			pose_data.set(pose);
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
	core::goal = { 3.f, 0.f, 0.f };

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
		//core::test_orientation();

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
	gazebo::msgs::Set(&msg, vel_data.get_pose());

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
		flctrl::obs_avoid(nearest_obs, vel_data);
		//flctrl::obs_avoid_simple(nearest_obs, vel_data);
	}
	else
	{
		state = goal_nav;
		//stop_vehicle();
		//test_orientation();
		
		// stop fuzzy navigation if goal is "reached"
		flctrl::goal_nav(pose_data, goal, vel_data);
		/* 
		if (pose_data.dist(goal) > 0.05) flctrl::goal_nav(pose_data, goal, vel_data);;
		else
		{
			stop_vehicle();
		}
		*/	
	}
}

void
core::stop_vehicle()
{
	core::vel_data.trans = 0.f;
	core::vel_data.ang   = 0.f;
}