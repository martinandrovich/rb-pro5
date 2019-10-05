#pragma once

#include <string>
#include <mutex>
#include <iostream>

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
	// constants
	
	const std::string PATH_ROOT = "";
	const std::string PATH_FUZZY_OBS_AVOID = PATH_ROOT + "assets/data/fuzzy-obs-avoid.fll";

	const std::string WNDW_CAMERA = "camera";
	const std::string WNDW_LIDAR  = "lidar";

	constexpr auto RUN_FREQ_MS = std::chrono::milliseconds(10);

	// enumerations

	enum ctrl_state_t
	{
		simple_nav,
		obs_avoid
	};

	// structures

	struct lidar_t
	{	
		std::mutex mutex;

		gazebo::msgs::LaserScan scan;

		float angle_min;
		float angle_max;
		float angle_increment;
		float range_min;
		float range_max;

		int32_t sec;
		int32_t nsec;
		int32_t nranges;
		int32_t nintensities;

		const cv::Mat&
		get_img_safe()
		{
			this->mutex.lock();

			// create image
			if (img.empty()) img = cv::Mat(height, width, CV_8UC3);

			// set to black
			img.setTo(0);

			// draw lidar data
			float px_per_m = 200 / range_max;

			for (int i = 0; i < nranges; i++)
			{
				auto angle = angle_min + i * angle_increment;
				auto range = std::min((float)scan.ranges(i), range_max);

				// draw lidar lines
				cv::Point2f pt_start(200.5f + range_min * px_per_m * std::cos(angle), 200.5f - range_min * px_per_m * std::sin(angle));
				cv::Point2f pt_end(200.5f + range * px_per_m * std::cos(angle), 200.5f - range * px_per_m * std::sin(angle));
				cv::line(img, pt_start * 16, pt_end * 16, cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
			}

			// draw robot
			cv::circle(img, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));

			// draw timestamp
			cv::putText(img, std::to_string(sec) + ":" + std::to_string(nsec), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));

			this->mutex.unlock();

			return this->img;
		}

	private:

		int32_t width = 400;
		int32_t height = 400;
		cv::Mat img;
	};

	struct pose_t
	{
		std::mutex mutex;
		
		struct pos 
		{
			float x;
			float y;
			float z;
		} position;

		struct orient 
		{
			float w;
			float x;
			float y;
			float z;
		} orientation;

		float dir() { return 4.f; }

		friend std::ostream&
		operator << (std::ostream& out, const pose_t& pose)
		{
			return out
				<< std::setprecision(2) << std::fixed << std::setw(6)
				<< pose.position.x << std::setw(6)
				<< pose.position.y << std::setw(6)
				<< pose.position.z << std::setw(6)
				<< pose.orientation.w << std::setw(6)
				<< pose.orientation.x << std::setw(6)
				<< pose.orientation.y << std::setw(6)
				<< pose.orientation.z << std::endl;
		} 
	};

	struct camera_t
	{
		std::mutex mutex;
		
		size_t width;
		size_t height;
		const char* data;

		const cv::Mat&
		get_img()
		{
			// check if function is mutex guarded
			assert(std::async(std::launch::async, [&]{ return this->mutex.try_lock(); })
				.get() == false && "the mutex is not locked.");

			auto img_temp = cv::Mat((int32_t)height, (int32_t)width, CV_8UC3, const_cast<char*>(this->data));
			this->img = img_temp.clone();

			cv::cvtColor(this->img, this->img, cv::COLOR_RGB2BGR);

			return this->img;
		}

		const cv::Mat&
		get_img_safe()
		{
			this->mutex.lock();
			this->get_img();
			this->mutex.unlock();

			return this->img;
		}

	private:
	
		cv::Mat img;
	};

	struct vel_t
	{
		std::mutex mutex;

		float dir;
		float rot;

		auto pose() { return ignition::math::Pose3d(this->dir, 0, 0, 0, 0, this->rot); }
	};

	struct obs_t
	{
		float dir;
		float dist;
	};
	
}