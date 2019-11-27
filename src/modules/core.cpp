#include "core.h"

// private declarations for core

namespace core
{

	// private members

	bool initialized = false;

	gazebo::transport::NodePtr node;
	gazebo::transport::SubscriberPtr sub_lidar;
	gazebo::transport::SubscriberPtr sub_camera;
	gazebo::transport::SubscriberPtr sub_pose;
	gazebo::transport::PublisherPtr pub_velcmd;
	gazebo::transport::PublisherPtr pub_world;
	gazebo::msgs::WorldControl ctrl_msg;

	lidar_t       	lidar_data;
	camera_t      	camera_data;
	pose_t        	pose_data;
	pose_t 			pose_estimate;
	vel_t         	vel_data;
	pos_t         	goal;
	marble_t      	nearest_marble;
	obs_list_t    	nearest_obs;
	marble_list_t 	marbles;

	pose_estimate_t pose_est_data(PATH_FLOOR_PLAN, FLOOR_PLAN_SCALE); 

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

	tune_morphology::morph_settings 
	tune_morphology_settings(const std::string& video_path);
}

// --------------------------------------------------------------------------------
// private method defintions for ::core
// --------------------------------------------------------------------------------

inline void
core::make_debug_data()
{
	constexpr auto WIDTH = 20;

	debug::dout << std::left      
		<< std::setw(WIDTH) << "State (flctrl):"    << std::left << flctrl::ctrl_state_names[(int)flctrl::state] << "\n"
		<< std::setw(WIDTH) << "Position:"          << std::left << core::pose_data.pos << "\n"
		<< std::setw(WIDTH) << "Orientation:"       << std::left << core::pose_data.orient << "\n"
		<< std::setw(WIDTH) << "Direction:"         << std::left << "y: " << core::pose_data.orient.yaw << "\n"
		<< std::setw(WIDTH) << "Velocity:"          << std::left << core::vel_data << "\n"
		<< "\n"
		<< std::setw(WIDTH) << "Goal:"              << std::left << core::goal << "\n"
		<< std::setw(WIDTH) << ""                   << std::left << "d: " << pose_data.dist(goal) << " | a: " << pose_data.dir(goal) << "\n"
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
		throw std::runtime_error(ERR_RE_INIT);

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

	// set mutable reset
	core::ctrl_msg.mutable_reset()->set_all(true);

	// publish control message
	core::pub_world->WaitForConnection();
	core::pub_world->Publish(core::ctrl_msg);

	// set goal
	core::goal = GOAL_POS;

	// set initialization status
	core::initialized = true;
}

void
core::run()
{
	
	// assert that system is initialized
	if (not core::initialized)
		throw std::runtime_error(ERR_NOT_INIT);
	
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

		// floor plane output
		cv::imshow(WNDW_ESTIMATE, pose_est_data.get_img(pose_data));

		// run main controller
		core::controller();

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
	// check velocity commands for NaN
	if(vel_data.is_nan())
		throw std::runtime_error(ERR_VELCMD_NAN);

	// convert pose to message; using the global velocity info
	static gazebo::msgs::Pose msg;
	gazebo::msgs::Set(&msg, vel_data.get_pose());

	// 
	pose_est_data.timing_start_t2();

	// publish the velocity command
	core::pub_velcmd->Publish(msg);

	// should be performed right after 
	pose_est_data.particle_filter(vel_data);

	// 
	pose_est_data.timing_start_t1();
}

void
core::controller()
{
	// localization
	if (USE_LOCALIZATION){}
		;//loc::update_pose(pose_data);

	// extract nearest obstacles
	nearest_obs["center"] = lidar_data.get_nearest_obs(pose_data.pos, { 0.38, -0.38 });
	nearest_obs["right"]  = lidar_data.get_nearest_obs(pose_data.pos, {-1.37, -1.76 });
	nearest_obs["left"]   = lidar_data.get_nearest_obs(pose_data.pos, { 1.76,  1.37 });
	nearest_obs["any"]    = lidar_data.get_nearest_obs(pose_data.pos);

	pose_est_data.get_lidar_data(lidar_data);

	// pose_estimate.pos.x = pose_est_data.posX;
	// pose_estimate.pos.y = pose_est_data.posY;
	// pose_estimate.orient.yaw = pose_est_data.yaw;

	// extract nearest marbles
	// marbles = camera_data.get_marbles();
	// nearest_marble = camera_data.get_nearest_marble();

	// path generation + AI
	;

	// fuzzy controller
	flctrl::run(nearest_obs, pose_data, goal, vel_data);
	// flctrl::run(nearest_obs, pose_estimate, goal, vel_data);


	// publish velocity command
	core::publish_velcmd();
}

void
core::stop_vehicle()
{
	core::vel_data.trans = 0.f;
	core::vel_data.ang   = 0.f;
}

void
core::test_run(const std::string& path_to_video_writer)
{
	cv::VideoWriter video_writer(PATH_ASSETS + path_to_video_writer, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, camera_data.get_img_size() );

	// assert that system is initialized
	if (not core::initialized)
		throw std::runtime_error(ERR_NOT_INIT);
	
	int key = 0;
	const int key_left = 81;
  	const int key_up = 82;
  	const int key_down = 84;
  	const int key_right = 83;  	
	const float scale_factor = 0.5;

	auto settings =  tune_morphology_settings("Marble_data_big_world.avi");

	// loop
	while (true)
	{
		// sleep
		std::this_thread::sleep_for(RUN_FREQ_MS);

		// acquire key input from opencv and draw any imshow() windows
		// close app if ESC key has been pressed
		
    	key = cv::waitKey(1);    	
		if(key == 27) break;		
	
		// show camera output
		cv::imshow(WNDW_CAMERA, camera_data.get_img());
		//video_writer.open();

		if(video_writer.isOpened())
		{
			video_writer.write(camera_data.get_img());		
			//std::cout << "The video has been opened" << std::endl;			
		}

		if ((key == key_up) && (core::vel_data.trans <= 1.2f))
		core::vel_data.trans += 0.05 * scale_factor;
		else if ((key == key_down) && (core::vel_data.trans >= -1.2f))
		core::vel_data.trans -= 0.05 * scale_factor;
		else if ((key == key_right) && (core::vel_data.ang <= 0.4f))
		core::vel_data.ang  += 0.05 * scale_factor;
		else if ((key == key_left) && (core::vel_data.ang  >= -0.4f))
		core::vel_data.ang  -= 0.05 * scale_factor;

		// publish velocity command
		core::publish_velcmd();

		// show debug information
		debug::show(&core::make_debug_data);

		// align windows
		core::align_windows();
	}	
	video_writer.release();

	// shutdown gazebo
	gazebo::client::shutdown();	

}


tune_morphology::morph_settings 
core::tune_morphology_settings(const std::string& video_path)
{
	return tune_morphology::choose_optimal_morph(PATH_ASSETS + video_path);
}
