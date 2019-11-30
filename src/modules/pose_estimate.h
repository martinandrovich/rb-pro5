#pragma once

#include <algorithm>
#include <mutex>
#include <vector>
#include <chrono>
#include <ctime>
#include <string>
#include <random>
#include <cmath>
#include <fstream>
#include <iostream>
#include <utility>
#include <future>
#include <iterator>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <types/pose.h>
#include <types/obs.h>
#include <types/lidar.h>

#include "../constants.h"

class pose_estimate_t
{

public:

	pose_estimate_t() = default;

	void
	init(cv::String filepath, float scalefactor_)
	{
		// :  scalefactor(scalefactor_), integral_mov(0), integral_dir(0)
		this->scalefactor = scalefactor_;
		this->integral_mov = 0;
		this->integral_dir = 0;

		img = cv::imread(filepath, cv::IMREAD_GRAYSCALE);

		if (img.empty())
			std::cerr << "Map failed to load" << std::endl;
		else
		{
			
			posX = 0;
			posY = 0;
			yaw = 0;

			// scale the image
			int scaleX = ((float)(img.cols * PTCLFILT_IMG_SCALE) / scalefactor);
			int scaleY = ((float)(img.rows * PTCLFILT_IMG_SCALE) / scalefactor);

			// resize picture to use it for measurements
			cv::resize(img, img_scaled, cv::Size(scaleX, scaleY), 1, 1, cv::INTER_NEAREST);

			yaxis = img_scaled.rows;
			xaxis = img_scaled.cols;

			// Draw corners in map
			for (size_t i = 0; i < (size_t)img_scaled.rows; i++)
				for (size_t j = 0; j < (size_t)img_scaled.cols; j++)
					if (i == 0 || i == (size_t)img_scaled.rows - 1 || j == (size_t)img_scaled.cols - 1 || j == 0)
						img_scaled.at<uchar>(i, j) = 0;

			// init LUT
			init_lookup_table();

			/*
			cv::Point marble(xaxis / 2 + 18.217300 * PTCLFILT_IMG_SCALE, yaxis / 2 - 0.878297 * PTCLFILT_IMG_SCALE );
			cv::circle(img_scaled, marble, 10, cv::Scalar(0, 0, 255) );
			marble = cv::Point(xaxis / 2 + 23.752800 * PTCLFILT_IMG_SCALE, yaxis / 2 - 0.59 * PTCLFILT_IMG_SCALE );
			cv::circle(img_scaled, marble, 10, cv::Scalar(0, 0, 255)     );
			marble = cv::Point(xaxis / 2 + 23.264400 * PTCLFILT_IMG_SCALE, yaxis / 2 - 18.303200 * PTCLFILT_IMG_SCALE );
			cv::circle(img_scaled, marble, 10, cv::Scalar(0, 0, 255) );
			*/

			cv::imwrite("picture_of_map.jpg", img_scaled);

			// genereate some particles
			static std::mt19937 generator;
			std::uniform_real_distribution<float> distribution_pox_x(100, xaxis - 100);
			std::uniform_real_distribution<float> distribution_pox_y(100, yaxis - 100);
			std::uniform_real_distribution<float> distribution_angle(0, 0);

			for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
			{
				particles[i].pos.x = distribution_pox_x(generator);
				particles[i].pos.y = distribution_pox_y(generator);
				particles[i].orient.yaw = 0;
				weights[i] = 1 / PTCLFILT_PARTICLES;
			}

			// init timer
			timing_start_t1();
		}
	};

	void
	timing_start_t1()
	{
		t1 = std::chrono::high_resolution_clock::now();
	}

	void
	timing_start_t2()
	{
		t2 = std::chrono::high_resolution_clock::now();
	}

	auto
	timing_get_time_ms()
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	}

	void
	get_lidar_data(lidar_t &lidar_data_)
	/*
	* Get the Lidar Data
	*/
	{
		pos_t data;
		auto temp = lidar_data_.get_vec_obs(data);
		int i = 0;

		if (ray_data.size() > LIDAR_MAX_RANGE)
			throw std::runtime_error(ERR_EXCEED_LIDAR_RANGE);

		if (ray_data.size() != LIDAR_MAX_RANGE)
		{
			std::cout << "Resized Ray Data to " << LIDAR_MAX_RANGE << std::endl;
			ray_data.clear();
			ray_data.resize(LIDAR_MAX_RANGE, temp[0]);
		}

		for (auto &temp_ : temp)
		{
			temp_.dist = temp_.dist * PTCLFILT_IMG_SCALE;
			ray_data[i] = temp_;
			i++;
		}
	}

	inline const cv::Mat
	get_img(pose_t &robot_pose)
	{
		//static int pic = 0;
		mutex.lock();
		std::future<void> t (std::async(std::launch::async, &pose_estimate_t::mean_pos, this) );
		auto img_copy = img_scaled.clone();
		cv::cvtColor(img_copy, img_copy, cv::COLOR_GRAY2BGR);

		// check for empty image
		if (img_copy.empty())
			img_copy = cv::Mat(200, 200, CV_8UC1);
		else
		{
			for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
			{
				if (particles[i].pos.y >= 0 && particles[i].pos.y < img_copy.rows && particles[i].pos.x >= 0 && particles[i].pos.x < img_copy.cols && img_copy.at<uchar>(particles[i].pos.y, particles[i].pos.x) != 0)
				{
					//img_copy.at<cv::Vec3b>(particles[i].pos.y, particles[i].pos.x) = cv::Vec3b(50,50,50);
					arrowedLine(img_copy, cv::Point(particles[i].pos.x, particles[i].pos.y), cv::Point(particles[i].pos.x + cos(particles[i].orient.yaw) * 10, particles[i].pos.y + sin(particles[i].orient.yaw) * 10), cv::Scalar(0,255,0));
				}
			}

			t.get();

			cv::Point mean_pos(mean_position.pos.x, mean_position.pos.y);
			cv::circle(img_copy, mean_pos, 5, cv::Scalar(255, 0, 0));

			cv::Point robot(xaxis / 2 + robot_pose.pos.x * PTCLFILT_IMG_SCALE, yaxis / 2 - robot_pose.pos.y * PTCLFILT_IMG_SCALE);
			cv::circle(img_copy, robot, 5, cv::Scalar(0, 0, 255));
			
			float orientation = 0;

			for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
			{
				orientation += particles[i].orient.yaw; 
			}

			posX = mean_position.pos.x - xaxis/2;
			posY = mean_position.pos.y - yaxis/2;
			posX /= PTCLFILT_IMG_SCALE;
			posY /= -PTCLFILT_IMG_SCALE;
			yaw = -orientation/PTCLFILT_PARTICLES;

			// std::string a = std::string("pictures/") + std::to_string(pic++) + std::string(".png");

			// cv::imwrite( a , img_copy);

			fpos.open("position.csv", std::ios::out | std::ios::app);
			fpos << mean_pos.x << " , "<< mean_pos.y << " , " <<  orientation/PTCLFILT_PARTICLES << " , " << robot.x << " , " << robot.y << " , " << robot_pose.orient.yaw << std::endl;
			fpos.close();
		}

		mutex.unlock();
		return img_copy;
	}

	void
	particle_filter(vel_t &vel_data)
	/*
	* The particle filter, it does everything to do.
	*/
	{
		static std::random_device rd{};
		static std::mt19937 gen{rd()};
		static std::normal_distribution<float> distx(0, PTCLFILT_POS_NOISE_SIGMA); // 0.4 0.5 0.6 1.2
		static std::normal_distribution<float> disty(0, PTCLFILT_POS_NOISE_SIGMA); // 0.4 0.5 0.6 1.2
		static std::normal_distribution<float> angle(0, PTCLFILT_ANG_NOISE_SIGMA); // 0.1

		// integrate the data
		integral_mov = vel_data.trans * ((float)timing_get_time_ms()) / 1000.0;
		integral_dir = vel_data.ang * ((float)timing_get_time_ms()) / 1000.0;

		// calculate for each particles the updates and add some noise to measurements

		for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
		{
			// Adding random noise to the particles
			particles[i].pos.x += integral_mov * cos(particles[i].orient.yaw) * PTCLFILT_IMG_SCALE + distx(gen);
			particles[i].pos.y += integral_mov * sin(particles[i].orient.yaw) * PTCLFILT_IMG_SCALE + disty(gen);
			particles[i].orient.yaw -= integral_dir + angle(gen);

			if (particles[i].orient.yaw >= M_PI)
				particles[i].orient.yaw = particles[i].orient.yaw - 2 * M_PI;
			else if (particles[i].orient.yaw <= -M_PI)
				particles[i].orient.yaw = 2 * M_PI - particles[i].orient.yaw;
		}
		
		std::vector<std::future<void>> t;
		std::pair<size_t, size_t> particlerange[PTCLFILT_THREADS];
		int temp = 0;

		for (size_t i = 0; i < PTCLFILT_THREADS; i++)
		{
			if (i == PTCLFILT_THREADS - 1)
				particlerange[i] = std::pair<int, int>(temp, PTCLFILT_PARTICLES);
			else
				particlerange[i] = std::pair<int, int>(temp, temp + PTCLFILT_PARTICLES / PTCLFILT_THREADS);
			temp += PTCLFILT_PARTICLES / PTCLFILT_THREADS + 1;
			t.push_back(std::async(std::launch::async, &pose_estimate_t::calc_weights, this, particlerange[i].first, particlerange[i].second));
		}
		for (auto &t_ : t)
			t_.get();

		//Normalize Weights
		normalize_weights();

		//Resample the particles
		resample();
	}


	float posX, posY, yaw;

private:
	void
	calc_weights(size_t par_start, size_t par_end)
	/*
	*   Calculate weights for which particles to choose later on, this is based on using the look up table, therefore 
	*   init_lookup_table() usage, (done in constructor as it is now). Weights are normalized.
	*/
	{
		int index = 0;
		size_t i = 0, j = 0, x = 0, y = 0;
		float likelihood = 0;

		auto map = img_scaled.clone();

		for (i = par_start; i < par_end; ++i)
		{

			// find the position where it fits
			auto it = std::find_if(angle.begin(), angle.end(),
								   [&](const auto &angle) { return std::abs(angle - particles[i].orient.yaw) <= PTCLFILT_DELTA; });

			// get index value

			if (((int)(it - angle.begin())) < (int)ray_data.size() / 2)
				index = (int)angle.size() - (int)(ray_data.size() / 2) + ((int)(it - angle.begin()));
			else
				index = ((it - angle.begin()) - ray_data.size() / 2);

			likelihood = 0;

			x = particles[i].pos.x;
			y = particles[i].pos.y;

			if (index < 0 || index >= angle.size()) 
			{
				std::cout << "ERROR OCCURED" << std::endl;
			}

			if (y >= 0 && y < map.rows && x >= 0 && x < map.cols)
			{
				// calculate likelihood
				index = (index + 1) % angle.size();
				for (j = 0; j < ray_data.size(); j++)
				{
					//trying this out
					likelihood += norm_pdf(ray_data[j].dist, lookuptable_mean[y][x][index], 3);
					index = (index + 1) % angle.size();
				}
			}

			weights[i] = likelihood;
		}
	}

	void
	resample()
	/*
	/ Bimodal Resampling is used, uniform sampling with weights.
	*/
	{
		//random index
		static std::random_device rd{};
		static std::mt19937 generator(rd());
		static std::uniform_real_distribution<float> distribution(0, 1);
		int index = PTCLFILT_PARTICLES * distribution(generator);

		float beta = 0;

		//get max weight
		float *weights_max = std::max_element(weights, weights + PTCLFILT_PARTICLES);

		//circular resampling
		for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
		{
			beta += distribution(generator) * 2 * (*weights_max);

			while (weights[index] < beta)
			{
				beta -= weights[index];
				index = (index + 1) % PTCLFILT_PARTICLES;
			}

			new_particles[i].pos.x = particles[index].pos.x;
			new_particles[i].pos.y = particles[index].pos.y;
			new_particles[i].orient.yaw = particles[index].orient.yaw;
		}

		// sampling with replacement therefore take values back
		for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
		{
			particles[i].pos.x = new_particles[i].pos.x;
			particles[i].pos.y = new_particles[i].pos.y;
			particles[i].orient.yaw = new_particles[i].orient.yaw;
		}
	}

	void
	normalize_weights()
	/*
	/ This is not parallelisede yet, however it is quite fast.
	*/
	{
		float sum = 0;
		for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
		{
			sum += weights[i];
		}

		for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
		{
			weights[i] /= sum;
		}
	}

	void
	init_lookup_table()
	/*

	*   Purpose is to init the tripple vector with the ray means from the picture.
	*/
	{
		float iterate = PTCLFILT_START_VAL;

		// angles get iterated
		angle.clear();
		dirs.clear();
		lookuptable_mean.clear();

		// init tripple vector
		lookuptable_mean = std::vector<std::vector<std::vector<float>>>(img_scaled.rows, std::vector<std::vector<float>>(img_scaled.cols, std::vector<float>(PTCLFILT_ANGLES, 0)));

		for (size_t i = 0; i < PTCLFILT_ANGLES; ++i)
		{
			angle.push_back(iterate);
			dirs.push_back(cv::Point2f(cos(angle[i]), sin(angle[i])));
			iterate -= PTCLFILT_DELTA;
		}

		iterations = 0;

		std::vector<std::future<void>> t;
		std::pair<size_t, size_t> yrange[PTCLFILT_THREADS];
		int temp = 0;
		for (size_t i = 0; i < PTCLFILT_THREADS; i++)
		{
			if (i == PTCLFILT_THREADS - 1)
				yrange[i] = std::pair<int, int>(temp, img_scaled.rows);
			else
				yrange[i] = std::pair<int, int>(temp, temp + img_scaled.rows / PTCLFILT_THREADS);
			temp += img_scaled.rows / PTCLFILT_THREADS + 1;
			t.push_back(std::async(std::launch::async, &pose_estimate_t::lut_table, this, yrange[i].first, yrange[i].second));
		}
		for (auto &t_ : t)
			t_.get();
	}

	void
	lut_table(const size_t ys, const size_t ye)
	/*
	* Effiecient calculation of table
	*/
	{
		cv::Point2f start(0, 0);
		cv::Point2f stop(0, 0);
		cv::Point2f end(0, 0);
		int j = 0;

		auto img = img_scaled.clone();

		// Rows
		for (size_t y = ys; y < ye; ++y)
		{
			// Columns
			for (size_t x = 0; x < (size_t)img_scaled.cols; ++x)
			{
				// If Row and Columns here are black.
				if (img.at<uchar>(cv::Point(x, y)) == 0)
				{
					for (size_t i = 0; i < angle.size(); i++)
					{
						lookuptable_mean[y][x][i] = 0.001f;
					}
				}
				else
				{
					for (size_t i = 0; i < dirs.size(); i++)
					{
						//This might seem counterintuative
						start = cv::Point2f(x, y);
						stop = cv::Point2f(x + PTCLFILT_LIDAR_SCALE * dirs[i].x, y + PTCLFILT_LIDAR_SCALE * dirs[i].y);
						cv::LineIterator it(img, start, stop, 8);
						j = 0;
						while (img.at<uchar>(it.pos()) == (uchar)255 && j < it.count)
						{
							end = it.pos();
							++it;
							++j;
						}
						lookuptable_mean[y][x][i] = sqrt(euclideanDist(start, end));
					}
				}
			}
			
			std::cout << iterations++ << std::endl;
		}
	}

	void
	mean_pos()
	/*
	* Calculate mean position
	*/
	{

		mean_position.pos.x         = 0;
		mean_position.pos.y         = 0;
		mean_position.orient.yaw    = 0;

		for (size_t i = 0; i < PTCLFILT_PARTICLES; i++)
		{
		   mean_position.pos.x      += particles[i].pos.x;
		   mean_position.pos.y      += particles[i].pos.y;
		   mean_position.orient.yaw += particles[i].orient.yaw;
		}

		mean_position.pos.x         /= PTCLFILT_PARTICLES;
		mean_position.pos.y         /= PTCLFILT_PARTICLES;
		mean_position.orient.yaw    /= PTCLFILT_PARTICLES;

	}

	inline float
	norm_pdf(float x, float m = 0, float s = 1)
	/*
	* Calculates the normal distribution fast.
	*/
	{
		static const float inv_sqrt_2pi = 0.3989422804014327;
		float a = (x - m) / s;
		return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
	}

	inline float
	euclideanDist(const cv::Point2f &a, const cv::Point2f &b)
	/*
	* Euclidean Distance rather than openCV's slow version.
	*/
	{
		cv::Point diff = a - b;
		return diff.x * diff.x + diff.y * diff.y;
	}

private:
	/*
	* Only variables below here
	*/

	cv::Mat img;
	cv::Mat img_scaled;

	std::chrono::system_clock::time_point t1;
	std::chrono::system_clock::time_point t2;

	float scalefactor;
	float integral_mov;
	float integral_dir;
	float xaxis, yaxis;
	int iterations;

	std::mutex mutex;

	pose_t particles[PTCLFILT_PARTICLES];
	pose_t new_particles[PTCLFILT_PARTICLES];
	pose_t mean_position;
	float weights[PTCLFILT_PARTICLES];

	std::fstream fpos;

	pose_t goal_;
	lidar_t lidar_data;
	std::vector<obs_t> ray_data;

	/*
	* Variables for look up table here!!
	*/

	std::vector<std::vector<std::vector<float>>> lookuptable_mean;
	std::vector<float> angle;
	std::vector<cv::Point2d> dirs;
};