#include "core.h"

// private declarations for core

namespace core
{

	// private members

	bool initialized = false;

	std::mutex cv_mutex;

	gazebo::transport::SubscriberPtr sub_lidar;
	gazebo::transport::PublisherPtr pub_movement;
	gazebo::transport::PublisherPtr pub_world;
	gazebo::msgs::WorldControl ctrl_msg;

	fl::Engine* fl_engine;

	lidar_t lidar_data;
	trajectory_t traj_data;

	// private methods
	
	void
	callback_lidar(ConstLaserScanStampedPtr& msg);

	void
	callback_camera(ConstImageStampedPtr& msg);

	void
	process_data();

	void
	publish_pose();

	void
	fuzzy_controller();

}

// --------------------------------------------------------------------------------
// private method defintions for ::core
// --------------------------------------------------------------------------------

void
core::callback_lidar(ConstLaserScanStampedPtr& msg)
{

	lidar_data.mutex.lock();

	lidar_data.angle_min = float(msg->scan().angle_min());
	lidar_data.angle_max = msg->scan().angle_max();
	lidar_data.angle_increment = float(msg->scan().angle_step());
	lidar_data.range_min = float(msg->scan().range_min());
	lidar_data.range_max = float(msg->scan().range_max());

	assert(lidar_data.nranges == lidar_data.nintensities);

	lidar_data.sec = msg->time().sec();
	lidar_data.nsec = msg->time().nsec();
	lidar_data.nranges = msg->scan().ranges_size();
	lidar_data.nintensities = msg->scan().intensities_size();

	lidar_data.mutex.unlock();

}

void
core::callback_camera(ConstImageStampedPtr& msg)
{
	return;
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

	// listen for gazebo topics (assign callbacks)

	// lidar
	core::sub_lidar = node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", core::callback_lidar);

	// setup gazebo publishers

	// movement
	core::pub_movement = node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

	// world
	core::pub_world = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

	//
	core::ctrl_msg.mutable_reset()->set_all(true);

	// publish control message
	core::pub_world->WaitForConnection();
	core::pub_world->Publish(core::ctrl_msg);

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
		// acquire key input from opencv; guarded by mutex
		core::cv_mutex.lock();
		auto key = cv::waitKey(1);
		core::cv_mutex.unlock();

		// check for ESC key
		if (key == 27) break;

		// process data
		core::process_data();

		// implement controller
		core::fuzzy_controller();

		// publish data
		core::publish_pose();

	}

	// shutdown gazebo
	gazebo::client::shutdown();
}

void
core::process_data()
{	
	lidar_data.mutex.lock();

	std::cout << lidar_data.nranges << std::endl;

	lidar_data.mutex.unlock();
}

void
core::publish_pose()
{
	// generate a pose
	ignition::math::Pose3d pose(traj_data.speed, 0, 0, 0, 0, traj_data.dir);

	// convert to a pose message
	gazebo::msgs::Pose msg;
	gazebo::msgs::Set(&msg, pose);

	// publish
	core::pub_movement->Publish(msg);
}

void
core::fuzzy_controller()
{
	// check enginge
	if (std::string status; not fl_engine->isReady(&status))
		throw fl::Exception("Fuzzylite engine is not ready:n" + status, FL_AT);

	// variables (loaded once)
	static fl::InputVariable* obs_dist_s     = fl_engine->getInputVariable("obs_dist_forward");
	static fl::InputVariable* obs_dist_r     = fl_engine->getInputVariable("obs_dist_right");
	static fl::InputVariable* obs_dist_l     = fl_engine->getInputVariable("obs_dist_left");
	static fl::OutputVariable* robot_dir     = fl_engine->getOutputVariable("robot_dir");
	static fl::OutputVariable* robot_speed   = fl_engine->getOutputVariable("robot_speed");

	// apply inputs
	// obs_dist_s->setValue(cloest_obs_front.range);
	// obs_dist_l->setValue(cloest_obs_left.range);
	// obs_dist_r->setValue(cloest_obs_right.range);

	// process data
	fl_engine->process();

	// export outputs
	core::traj_data = { (float)robot_dir->getValue(), (float)robot_speed->getValue() };
}
