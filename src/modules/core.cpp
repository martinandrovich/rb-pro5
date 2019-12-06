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
	gazebo::transport::PublisherPtr  pub_velcmd;
	gazebo::transport::PublisherPtr  pub_world;
	gazebo::msgs::WorldControl ctrl_msg;

	int             key = 0;

	lidar_t         lidar_data;
	camera_t        camera_data;
	pose_t          pose_data;
	pose_t          pose_estimate;
	vel_t           vel_data;
	pos_t           goal;
	marble_t        nearest_marble;
	obs_list_t      nearest_obs;
	marble_list_t   marbles;
	pose_estimate_t pose_est_data;
	cv::Mat         img_camera;
	// pose_estimate_t pose_est_data(PATH_FLOOR_PLAN, FLOOR_PLAN_SCALE); 

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
	callback_test(const boost::shared_ptr<gazebo::msgs::Any const>& msg);

	void
	publish_velcmd();

	void
	controller();

	void
	marble_detect();

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

	// initialize particle filter
	if (USE_PARTICLE_FILTER)
		pose_est_data.init(PATH_FLOOR_PLAN, FLOOR_PLAN_SCALE); 

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
		key = cv::waitKey(1);

		// close app if ESC key has been pressed
		if (key == 27) break;

		// show lidar outout
		cv::imshow(WNDW_LIDAR, lidar_data.get_img());
		
		// show camera output
		img_camera = camera_data.get_img();
		cv::imshow(WNDW_CAMERA, img_camera);

		// show particle filter output (if enabled)
		if (USE_PARTICLE_FILTER)
			cv::imshow(WNDW_PTCLFILT, pose_est_data.get_img(pose_data));

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
	if (USE_PARTICLE_FILTER)
		pose_est_data.timing_start_t2();

	// publish the velocity command
	core::pub_velcmd->Publish(msg);

	// should be performed right after 
	if (USE_PARTICLE_FILTER)
		pose_est_data.particle_filter(vel_data);

	// 
	if (USE_PARTICLE_FILTER)
		pose_est_data.timing_start_t1();
}

void
core::controller()
{
	// extract nearest obstacles
	nearest_obs["center"] = lidar_data.get_nearest_obs(pose_data.pos, { 0.38, -0.38 });
	nearest_obs["right"]  = lidar_data.get_nearest_obs(pose_data.pos, {-1.37, -1.76 });
	nearest_obs["left"]   = lidar_data.get_nearest_obs(pose_data.pos, { 1.76,  1.37 });
	nearest_obs["any"]    = lidar_data.get_nearest_obs(pose_data.pos);
	
	// marble detection
	if (USE_MARBLE_DETECT)
		marble_detect();

	// particle filter
	if (USE_PARTICLE_FILTER)
	{
		pose_est_data.get_lidar_data(lidar_data);

		pose_estimate.pos.x = pose_est_data.posX;
		pose_estimate.pos.y = pose_est_data.posY;
		pose_estimate.orient.yaw = pose_est_data.yaw;
	}

	// path generation + AI
	;

	// robot control
	// using manual or fuzzy w/ global pose or estimated pose
	if (USE_MANUAL_CONTROL)
	{
		if ((key == 'w') && (core::vel_data.trans <= 1.2f))
			vel_data.trans += 0.05;

		else
		if ((key == 's') && (vel_data.trans >= -1.2f))
			vel_data.trans -= 0.05;

		else
		if ((key == 'd') && (vel_data.ang <= 0.4f))
			vel_data.ang  += 0.05;

		else
		if ((key == 'a') && (vel_data.ang  >= -0.4f))
			vel_data.ang  -= 0.05;
	}
	else
	if (USE_PARTICLE_FILTER && USE_LOCALIZATION)
	{
		flctrl::run(nearest_obs, pose_estimate, goal, vel_data);
	}
	else
	{
		flctrl::run(nearest_obs, pose_data, goal, vel_data);
	}

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
core::test_run(const std::string& video_filename)
{

	// assert that system is initialized
	if (not core::initialized)
		throw std::runtime_error(ERR_NOT_INIT);
	
	const int key_left = 'a';
  	const int key_up = 'w';
  	const int key_down = 's';
  	const int key_right = 'd';  	
	const float scale_factor = 0.5;

	// if video already recorded, perform marble detection
	if (file_exists(PATH_ASSETS + video_filename))
	{
		// perform tuning
		auto settings =  tune_morphology_settings(video_filename);
		return;
	}
	// otherwise record video using control

	// open video writer to record video
	cv::VideoWriter video_writer(PATH_ASSETS + video_filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, camera_data.get_img_size() );

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

		if ((key == 'w') && (core::vel_data.trans <= 2.4f))
		core::vel_data.trans += 0.05 * scale_factor;
		else if ((key == 's') && (core::vel_data.trans >= -2.4f))
		core::vel_data.trans -= 0.05 * scale_factor;
		else if ((key == 'd') && (core::vel_data.ang <= 0.4f))
		core::vel_data.ang  += 0.05 * scale_factor;
		else if ((key == 'a') && (core::vel_data.ang  >= -0.4f))
		core::vel_data.ang  -= 0.05 * scale_factor;

		// publish velocity command
		core::publish_velcmd();

		// show debug information
		debug::show(&core::make_debug_data);

		// align windows
		core::align_windows();
	}

	// end recording
	video_writer.release();
	cv::destroyAllWindows();

	// shutdown gazebo
	gazebo::client::shutdown();
}

inline void
core::marble_detect()
{

	std::vector<cv::Vec3f> vec_circles;

	auto gauss_ksize = 3;
	auto gauss_sigma = 4;
	auto morph_ksize = 1;
	auto morph_op = cv::MORPH_BLACKHAT;
	auto hough_upper_tresh = 124;
	auto hough_center_tresh = 19;
	auto hough_min_radius = 21;

	cv::Mat img = img_camera.clone();
	cvtColor(img, img, cv::COLOR_BGR2GRAY);

	static cv::Mat morph_elm = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(morph_ksize, morph_ksize));

	// apply blur and do the morphology with the chosen settings.
	cv::GaussianBlur(img, img, cv::Size(gauss_ksize, gauss_ksize), gauss_sigma);
	cv::morphologyEx(img, img, morph_op, morph_elm);

	// apply HoughCircle transform and save all circles found to a vector
	cv::HoughCircles(img, vec_circles, cv::HOUGH_GRADIENT, 1, 100, hough_upper_tresh, hough_center_tresh, hough_min_radius, 0);

	// Draw the circles detected
	for (size_t i = 0; i < vec_circles.size(); i++ )
	{
		auto center = cv::Point(cvRound(vec_circles[i][0]), cvRound(vec_circles[i][1]));
		auto radius = cvRound(vec_circles[i][2]);
		auto dist   = 0.5f * 277.0f / radius;
		auto text = "marble: r: " + std::to_string(radius) + " | d: " + std::to_string(dist);

		// output text on image
		cv::putText(
			img_camera,
			text,
			cv::Point(10, 20),
			cv::FONT_HERSHEY_SIMPLEX,
			0.5,
			CV_RGB(255, 255, 255),
			1
		);

		// circle center
		cv::circle(img_camera, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

		// circle outline
		cv::circle(img_camera, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
	}

	// override output image
	cv::imshow(WNDW_CAMERA, img_camera);
}


tune_morphology::morph_settings 
core::tune_morphology_settings(const std::string& video_path)
{
	return tune_morphology::choose_optimal_morph(PATH_ASSETS + video_path);
}
