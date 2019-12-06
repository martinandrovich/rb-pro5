# pragma once
#include <mutex>
#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>

#include "pose.h"
#include "obs.h"

// --------------------------------------------------------------------------------
// declarations for ::lidar_t
// --------------------------------------------------------------------------------

class lidar_t
{
public:

	struct ray_t
	{
		float angle, range;
	};

	struct range_t
	{
		float from, to;
		friend bool operator < (const range_t& lhs, const range_t& rhs) { return (lhs.from < rhs.from); }
	};

	void
	set(ConstLaserScanStampedPtr& msg);

	std::vector<obs_t>
	get_vec_obs(const pos_t& robot_pos);

	obs_t
	get_nearest_obs(const pos_t& robot_pos);

	obs_t
	get_nearest_obs(const pos_t& robot_pos, const range_t& range);

	const cv::Mat&
	get_img();

private:

	std::mutex mutex;

	float angle_min;
	float angle_max;
	float angle_increment;
	float range_min;
	float range_max;

	int32_t sec;
	int32_t nsec;
	int32_t num_ranges;
	int32_t num_intensities;

	float px_per_m;

	int32_t img_width  = 400;
	int32_t img_height = 400;
	cv::Mat img;

	std::vector<ray_t> vec_rays;
	std::map<range_t, obs_t> nearest_obs_cache;
};

// --------------------------------------------------------------------------------
// definitions for ::lidar_t
// --------------------------------------------------------------------------------

inline void
lidar_t::set(ConstLaserScanStampedPtr& msg)
{
	mutex.lock(); // CRITICAL SECTION BEGIN

	const auto& scan = msg->scan();
	const auto& time = msg->time();

	// data
	this->angle_min        = float(scan.angle_min());
	this->angle_max        = scan.angle_max();
	this->angle_increment  = float(scan.angle_step());
	this->range_min        = float(scan.range_min());
	this->range_max        = float(scan.range_max());
	this->num_ranges       = scan.ranges_size();
	this->num_intensities  = scan.intensities_size();
	this->sec              = time.sec();
	this->nsec             = time.nsec();

	// everything okay?
	assert(num_ranges == num_intensities);

	// popuate vector of rays
	this->vec_rays.clear();

	for (int i = 0; i < this->num_ranges; ++i)
	{
		auto angle = angle_min + i * angle_increment;
		auto range = std::min((float)scan.ranges(i), range_max);

		this->vec_rays.emplace_back(ray_t{angle, range});
	}

	mutex.unlock(); // CRITICAL SECTION END
}

inline std::vector<obs_t>
lidar_t::get_vec_obs(const pos_t& robot_pos)
{
	std::vector<obs_t> vec_obs;
	
	mutex.lock(); // CRITICAL SECTION BEGIN

	for(const auto& ray : this->vec_rays)
	{
		auto dir   = ray.angle;
		auto dist  = ray.range;
		auto pos_x = std::sin(dir) / dist;
		auto pos_y = std::cos(dir) / dist;
		auto pos_z = robot_pos.z;

		vec_obs.emplace_back(dir, dist, pos_t{pos_x, pos_y, pos_z});
	}

	mutex.unlock(); // CRITICAL SECTION END

	return vec_obs;
}

inline obs_t
lidar_t::get_nearest_obs(const pos_t& robot_pos)
{
	auto vec_obs = get_vec_obs(robot_pos);

	if (vec_obs.size() == 0) return obs_t();
	
	auto nearest_obs =  *std::min_element(vec_obs.begin(), vec_obs.end(), [](auto& a, auto& b){
		return a.dist < b.dist;
	});

	nearest_obs_cache[range_t{INFINITY, INFINITY}] = nearest_obs;

	return nearest_obs;
}

inline obs_t
lidar_t::get_nearest_obs(const pos_t& robot_pos, const range_t& range)
{
	// get vector of obstacles
	auto vec_obs = get_vec_obs(robot_pos);
	if (vec_obs.size() == 0) return obs_t();

	// find min element within given angle range
	obs_t nearest_obs_min;

	for (const auto& obs : vec_obs)
	{
		if (obs.dir < range.from && obs.dir > range.to && nearest_obs_min.dir > obs.dir)
			nearest_obs_min = obs;
	}

	// store element into map
	nearest_obs_cache[range] = nearest_obs_min;

	return nearest_obs_min;
}

inline const cv::Mat&
lidar_t::get_img()
{

	// create image
	if (this->img.empty())
		this->img = cv::Mat(img_height, img_width, CV_8UC3);

	// set image to black
	img.setTo(0);

	// pixels per meter
	this->px_per_m = (200 / this->range_max);

	mutex.lock(); // CRITICAL SECTION BEGIN

	// draw lidar data
	for (const auto& ray : vec_rays)
	{
		cv::Point2f pt_start(200.5f + range_min * px_per_m * std::cos(ray.angle), 200.5f - range_min * px_per_m * std::sin(ray.angle));
		cv::Point2f pt_end(200.5f + ray.range * px_per_m * std::cos(ray.angle), 200.5f - ray.range * px_per_m * std::sin(ray.angle));
		cv::line(img, pt_start * 16, pt_end * 16,  cv::Scalar(255, 255, 255, 255), 1, cv::LINE_AA, 4);
	}

	// draw (cached) obstacles
	for (auto& [key, obs] : nearest_obs_cache)
	{
		cv::Point2f pt_start(200.5f + range_min * px_per_m * std::cos(obs.dir), 200.5f - range_min * px_per_m * std::sin(obs.dir));
		cv::Point2f pt_end(200.5f + obs.dist * px_per_m * std::cos(obs.dir), 200.5f - obs.dist * px_per_m * std::sin(obs.dir));
		cv::line(img, pt_start * 16, pt_end * 16, cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA, 4);
	}

	// draw robot
	cv::circle(img, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));

	// draw timestamp
	cv::putText(img, std::to_string(sec) + ":" + std::to_string(nsec), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));

	mutex.unlock(); // CRITICAL SECTION END

	return this->img;
}