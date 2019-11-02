# pragma once

#include <mutex>
#include <map>

#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>

// --------------------------------------------------------------------------------
// declarations for ::marble_t
// --------------------------------------------------------------------------------

struct marble_t
{
	float whatev = 0.f;
};

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

	cv::Size 
	get_img_size();	

private:
	
	std::mutex mutex;
	
	const char* data;

	cv::Mat img;
	size_t  img_width;
	size_t  img_height;
	
};

// --------------------------------------------------------------------------------
// redefinitions
// --------------------------------------------------------------------------------

using marble_list_t = std::map<std::string, marble_t>;

// --------------------------------------------------------------------------------
// definitions for ::camera_t
// --------------------------------------------------------------------------------

inline void
camera_t::set(ConstImageStampedPtr& msg)
{
	this->mutex.lock(); // CRITICAL SECTION BEGIN

	this->img_width  = msg->image().width();
	this->img_height = msg->image().height();
	this->data       = msg->image().data().c_str();

	this->mutex.unlock(); // CRITICAL SECTION END
}

inline const cv::Mat&
camera_t::get_img()
{	
	mutex.lock(); // CRITICAL SECTION BEGIN

	auto img_temp = cv::Mat(this->img_height, this->img_width, CV_8UC3, const_cast<char*>(this->data));
	cv::cvtColor(img_temp, img, cv::COLOR_RGB2BGR);

	mutex.unlock(); // CRITICAL SECTION END

	// check for empty image
	if (this->img.empty())
		this->img = cv::Mat(200, 200, CV_8UC3);

	return img;
}

inline cv::Size 
camera_t::get_img_size()
{	
	return {(int)this->img_width, (int)this->img_height};
}	
