#include "core.h"

// private declarations for core

namespace core
{

	// private members

	gazebo::transport::SubscriberPtr sub_lidar;
	gazebo::transport::PublisherPtr pub_movement;
	gazebo::transport::PublisherPtr pub_world;
	gazebo::msgs::WorldControl ctrl_msg;

	bool initialized = false;
	std::mutex cv_mutex;

	// private methods
	
	void
	callback_lidar(ConstLaserScanStampedPtr& msg);

	void
	callback_camera(ConstImageStampedPtr& msg);

	void
	publish_pos();

	void
	fuzzy_controller();

}

// --------------------------------------------------------------------------------
// private method defintions for ::core
// --------------------------------------------------------------------------------

void
core::callback_lidar(ConstLaserScanStampedPtr& msg)
{
	return;
}

void
core::callback_camera(ConstImageStampedPtr& msg)
{
	return;
}

void
core::publish_pos()
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

	// test
	core::data::lidar_t lidar_data;

	// set initialization status
	core::initialized = true;
}

void
core::run()
{
	
	// assert that system is initialized
	if (!core::initialized)
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

		// implement controller
		core::fuzzy_controller();

		// log debug data
		;

	}

	// shutdown gazebo
  	gazebo::client::shutdown();
}

void
core::fuzzy_controller()
{
	// just testing
	core::data::nearst_obs_t nearest_obs(10,10);

	return;
}
