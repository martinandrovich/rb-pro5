# pragma once

#include <mutex>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>

//#include "../constants.h"

// --------------------------------------------------------------------------------
// declarations for ::camera_t
// --------------------------------------------------------------------------------

class camera_t
{
public:

	void
	set(ConstImageStampedPtr& msg);

	const cv::Mat&
	get_img();	

private:
	
	std::mutex mutex;
	
	const char* data;

	cv::Mat img;
	size_t  img_width;
	size_t  img_height;
	
};

// --------------------------------------------------------------------------------
// definitions for ::camera_t
// --------------------------------------------------------------------------------

inline const cv::Mat&
camera_t::get_img()
{	
	mutex.lock(); // CRITICAL SECTION BEGIN

	auto img_temp = cv::Mat(this->img_height, this->img_width, CV_8UC3, const_cast<char*>(this->data));
	img = img_temp.clone();

	cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

	mutex.unlock(); // CRITICAL SECTION END

	return img;
}