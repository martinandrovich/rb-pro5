#pragma once

#include <iostream>
#include <ostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <functional>
#include <algorithm>
#include <future>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <fl/Headers.h>

#include "../constants.h"

#include "types/obs.h"
#include "types/pose.h"
#include "types/lidar.h"
#include "types/camera.h"
#include "types/morph_settings.h"

#include "debug.h"
#include "utils.h"
#include "flctrl.h"
#include "pathgen.h"

// public declarations for core

namespace core
{
	void
	init(int argc, char** argv);

	void
	run();

	void
	test_run(const std::string& path_to_video_writer);
}