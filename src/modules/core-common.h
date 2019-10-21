#pragma once

#include <cmath>
#include <string>
#include <mutex>
#include <iostream>
#include <algorithm>
#include <future>
#include <map>

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
	const std::string PATH_FUZZY_SIMPLE_NAVIGATOR = PATH_ROOT + "assets/data/simpleNavigator.fll";
	const std::string PATH_FONT_CONSOLAS = PATH_ROOT + "assets/data/consolas.ttf";

	constexpr auto    RUN_FREQ_MS = std::chrono::milliseconds(10); // ms
	constexpr auto    MAX_DIST_TO_OBSTACLE = 0.2f; // meters
	constexpr auto    LIDAR_RANGE_LIMIT = 10; // number of rays
	//constexpr auto    LIDAR_RANGE_F = { 0.19, -0.19 }; // radians
	//constexpr auto    LIDAR_RANGE_L = { 0.19, -0.19 }; // radians
	//constexpr auto    LIDAR_RANGE_R = { 0.19, -0.19 }; // radians
	constexpr auto    FUZZY_SCALING_FACTOR = 0.50f;
	
	const std::string WNDW_CAMERA   = "camera";
	const std::string WNDW_LIDAR    = "lidar";
	const std::string WNDW_DEBUG    = "debug";
	const auto		  WNDW_HANDLES  = { WNDW_DEBUG, WNDW_LIDAR, WNDW_CAMERA };
	const size_t      WNDW_WIDTHS[] = { 700, 400, 200 };
	constexpr size_t  WNDW_ORIGIN[] = { 50, 50 };
	constexpr auto    WNDW_MARGIN   = 20;

	// enumerations

	enum ctrl_state_t
	{
		goal_nav,
		obs_avoid
	};

	constexpr std::string_view ctrl_state_names[] =
	{
		"goal navigator",
		"obstacle avoidance"
	};

	// structures

	struct pos_t 
	{
		float x, y, z;

		friend inline bool
		operator == (const pos_t& lhs, const pos_t& rhs)
		{
			return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
		}

		friend std::ostream&
		operator << (std::ostream& out, const pos_t& obj)
		{
			return out << "x: " << obj.x << " | y: " << obj.y << " | z: " << obj.z;
		}
	};

	struct orient_t
	{
		float w, x, y, z;

		friend inline bool
		operator == (const orient_t& lhs, const orient_t& rhs)
		{
			return (lhs.w == rhs.w && lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
		}

		friend std::ostream&
		operator << (std::ostream& out, const orient_t& obj)
		{
			return out << "w: " << obj.w << "x: " << obj.x << " | y: " << obj.y << " | z: " << obj.z;
		}
	};

	struct vel_t
	{
		float trans = 0.f;
		float ang   = 0.f;

		auto pose() { return ignition::math::Pose3d(this->trans, 0, 0, 0, 0, this->ang); }

		friend std::ostream&
		operator << (std::ostream& out, const vel_t& obj)
		{
			//return out << "trans: " << obj.trans << " | ang: " << obj.ang;
			return out << "x: " << obj.trans << " | w: " << obj.ang;
		}
	};

	struct pose_t
	{
		std::mutex mutex;
		
		pos_t pos;
		orient_t orient;

		float dir(const pos_t& other) 
		{
			std::lock_guard<std::mutex> lock(this->mutex); 		
			float dir = atan2(other.y - this->pos.y, other.x - this->pos.x); 
			float dif_in_orientation =  dir - this->orient.z;
			return dif_in_orientation;  
		}

		float dist(const pos_t& other) 
		{
			std::lock_guard<std::mutex> lock(this->mutex); 	 
			float dist = sqrt(pow(other.y - this->pos.y, 2) + pow( other.x - this->pos.x, 2));			
			return dist;
		}

		const pos_t& get_pos(){this->mutex.lock(); pos_t cpy = this->pos; this->mutex.unlock(); return std::move(cpy); }

		friend std::ostream&
		operator << (std::ostream& out, const pose_t& obj)
		{
			return out
				<< std::setprecision(2) << std::fixed << std::setw(6)
				<< obj.pos.x << std::setw(6)
				<< obj.pos.y << std::setw(6)
				<< obj.pos.z << std::setw(6)
				<< obj.orient.w << std::setw(6)
				<< obj.orient.x << std::setw(6)
				<< obj.orient.y << std::setw(6)
				<< obj.orient.z << std::endl;
		} 
	};

	struct obs_t
	{
		obs_t() : dir(INFINITY), dist(INFINITY), pos(pos_t{0, 0, 0}) {}
		obs_t(float dir, float dist, pos_t pos) : dir(dir), dist(dist), pos(pos) {}

		float dir;
		float dist;
		pos_t pos;

		friend inline bool
		operator == (const obs_t& lhs, const obs_t& rhs)
		{
			return (lhs.pos == rhs.pos);
		}

		friend std::ostream&
		operator << (std::ostream& out, const obs_t& obj)
		{
			return out << "a: " << obj.dir << " | d: " << obj.dist << " @ " << obj.pos;
		}
	};

	struct lidar_t
	{	
		std::mutex mutex;

		struct ray_t   { float angle, range; };
		struct range_t
		{
			float from,  to;
			friend bool operator < (const range_t& lhs, const range_t& rhs) { return (lhs.from < rhs.from); }
		};

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

		void
		set(ConstLaserScanStampedPtr& msg)
		{
			mutex.lock(); // CRITICAL SECTION BEGIN

			const auto& scan = msg->scan();
			const auto& time = msg->time();

			// data
			angle_min        = float(scan.angle_min());
			angle_max        = scan.angle_max();
			angle_increment  = float(scan.angle_step());
			range_min        = float(scan.range_min());
			range_max        = float(scan.range_max());
			num_ranges       = scan.ranges_size();
			num_intensities  = scan.intensities_size();
			sec              = time.sec();
			nsec             = time.nsec();

			px_per_m = 200 / range_max;

			// everything okay?
			assert(num_ranges == num_intensities);

			// popuate vector of rays
			vec_rays.clear();

			for (int i = 0 + LIDAR_RANGE_LIMIT; i < num_ranges - LIDAR_RANGE_LIMIT; i++)
			{
				auto angle = angle_min + i * angle_increment;
				auto range = std::min((float)scan.ranges(i), range_max);

				vec_rays.emplace_back(ray_t{angle, range});
			}

			mutex.unlock(); // CRITICAL SECTION END
		}

		std::vector<ray_t>
		get_vec_rays()
		{
			// scope based mutex; unlocks when in goes out of scope
			std::lock_guard<std::mutex> lock(this->mutex);

			// return copy
			return vec_rays; 
		}

		std::vector<obs_t>
		get_vec_obs(pos_t& robot_pos)
		{
			std::vector<obs_t> vec_obs;
			
			mutex.lock(); // CRITICAL SECTION BEGIN

			for(const auto& ray : vec_rays)
			{
				auto dir   = ray.angle;
				auto dist  = ray.range;
				auto pos_x = std::sin(dir) / dist;
				auto pos_y = std::cos(dir) / dist;
				auto pos_z = robot_pos.z;

				vec_obs.emplace_back(dir, dist, pos_t{pos_x, pos_y, pos_z});
			}

			mutex.unlock(); // CRITICAL SECTION END

			return std::move(vec_obs);
		}

		inline obs_t
		get_nearest_obs(pos_t& robot_pos)
		{
			auto vec_obs = get_vec_obs(robot_pos);

			if (vec_obs.size() == 0) return obs_t();
			
			nearest_obs =  *std::min_element(vec_obs.begin(), vec_obs.end(), [](auto& a, auto& b){
				return a.dist < b.dist;
			});

			return nearest_obs;
		}

		inline obs_t
		get_nearest_obs(pos_t& robot_pos, range_t range)
		{
			// get vector of obstacles
			auto vec_obs = get_vec_obs(robot_pos);
			if (vec_obs.size() == 0) return obs_t();

			// find min element within given range
			obs_t nearest_obs_min;

			for (const auto& obs : vec_obs)
			{
				if (obs.dir < range.from && obs.dir > range.to && nearest_obs_min.dir > obs.dir)
					nearest_obs_min = obs;
			}

			// store element into map
			nearest_obs_range[range] = nearest_obs_min;

			return nearest_obs_min;
		}

		const cv::Mat&
		get_img()
		{

			// create image
			if (img.empty()) img = cv::Mat(img_height, img_width, CV_8UC3);

			// set to black
			img.setTo(0);

			mutex.lock(); // CRITICAL SECTION BEGIN

			// draw lidar data
			for (const auto& ray : vec_rays)
			{
				auto line_color = (ray.range == nearest_obs.dist && ray.angle == nearest_obs.dir)
					? cv::Scalar(0, 0, 255, 255) : cv::Scalar(255, 255, 255, 255);

				cv::Point2f pt_start(200.5f + range_min * px_per_m * std::cos(ray.angle), 200.5f - range_min * px_per_m * std::sin(ray.angle));
				cv::Point2f pt_end(200.5f + ray.range * px_per_m * std::cos(ray.angle), 200.5f - ray.range * px_per_m * std::sin(ray.angle));
				cv::line(img, pt_start * 16, pt_end * 16, line_color, 1, cv::LINE_AA, 4);
			}

			// draw (cached) obstacles
			for (auto& [key, obs] : nearest_obs_range)
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

	private:

		int32_t img_width  = 400;
		int32_t img_height = 400;
		cv::Mat img;

		std::vector<ray_t> vec_rays;
		obs_t nearest_obs;
		std::map<range_t, obs_t> nearest_obs_range;
	};

	struct camera_t
	{
		std::mutex mutex;
		
		size_t  img_width;
		size_t  img_height;
		const char* data;

		const cv::Mat&
		get_img_unsafe()
		{
			// check if function is mutex guarded
			assert(std::async(std::launch::async, [&]{ return mutex.try_lock(); })
				.get() == false && "the mutex is not locked.");

			auto img_temp = cv::Mat((int32_t)img_height, (int32_t)img_width, CV_8UC3, const_cast<char*>(data));
			img = img_temp.clone();

			cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

			return img;
		}

		const cv::Mat&
		get_img()
		{
			mutex.lock(); // CRITICAL SECTION BEGIN
			get_img_unsafe();
			mutex.unlock(); // CRITICAL SECTION END

			return img;
		}

	private:
		cv::Mat img;
	};


	// redefinitions
	
	using obs_list_t = std::map<std::string, obs_t>;
}