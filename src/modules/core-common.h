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

	// structures

	struct lidar_t
	{
		std::mutex mutex;

		float angle_min;
		float angle_max;
		float angle_increment;
		float range_min;
		float range_max;

		int32_t sec;
		int32_t nsec;
		int32_t nranges;
		int32_t nintensities;
	};

	struct trajectory_t
	{
		float dir;
		float speed;
	};

	struct nearst_obs_t
	{
		nearst_obs_t(float dir_delta, float range) : dir_delta(dir_delta), range(range) {};

		float dir_delta;
		float range;
	};	
}