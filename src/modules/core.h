#pragma once

#include <iostream>
#include <ostream>
#include <math.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <functional>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <fl/Headers.h>

#include "core-common.h"
#include "debug.h"

// public declarations for core

namespace core
{
	void
	init(int argc, char** argv);

	void
	run();
}