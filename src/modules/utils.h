#pragma once

#include <string>
#include <filesystem>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>

#include "core-common.h"

// public declarations for utils

namespace utils
{
	std::string
	exec_path();
}