#pragma once

#include <string>
#include <mutex>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>

// --------------------------------------------------------------------------------
// common declarations for ::core
// --------------------------------------------------------------------------------

namespace core 
{
	// constant expressions
	
	constexpr std::string_view filepath = "/root/da.jpg";

	// whatever else
	
}

// --------------------------------------------------------------------------------
// common declarations for ::core::data
// --------------------------------------------------------------------------------

namespace core { namespace data
{

	struct lidar_t
	{
		std::mutex mutex;
		ConstLaserScanStampedPtr data;
	};

	struct nearst_obs_t
	{
		nearst_obs_t(float dir_delta, float range) : dir_delta(dir_delta), range(range) {};

		float dir_delta;
		float range;
	};

}}