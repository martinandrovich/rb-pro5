#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <functional>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../constants.h"
#include "types/pose.h"

namespace utils
{
	cv::Mat
	load_img(const std::string& filepath, cv::ImreadModes mode = cv::IMREAD_UNCHANGED);

	void
	show_img(const std::string& name, const cv::Mat& img, bool close = true);

	void
	scale_image(cv::Mat& img, float scale);

	float
	scale_image(cv::Mat& img, const dim_t& dim_world, float meter_per_px);

	template<typename D = std::chrono::microseconds, typename F> float
	benchmark(F lambda, bool print = true);
}

inline cv::Mat
load_img(const std::string& filepath, cv::ImreadModes mode = cv::IMREAD_UNCHANGED)
{
	cv::Mat img = cv::imread(filepath, mode);

	if (img.empty())
		throw std::runtime_error(ERR_IMG_EMPTY);

	return img;
}

inline void
show_img(const std::string& name, const cv::Mat& img, bool close = true)
{
	if (img.empty())
		throw std::runtime_error(ERR_IMG_EMPTY);

	cv::imshow(name, img);
	cv::waitKey();

	if (close)
		cv::destroyWindow(name);
}

inline float
scale_img(cv::Mat& img, const dim_t& dim_world, const float meter_per_px)
{
	// compute scaling factors
	auto scale_width  = dim_world.width  / (meter_per_px * img.cols);
	auto scale_height = dim_world.height / (meter_per_px * img.rows);
	auto scale        = (scale_width + scale_height) / 2.f;

	cv::resize(img, img, cv::Size(), scale_width, scale_height, cv::INTER_NEAREST);

	return scale;
}

inline void
iterate_mat(cv::Mat& img, std::function<void(cv::Point&, uchar&)> callback)
{
	for (size_t row = 0; row < (size_t)img.rows; ++row)
	{
		for (size_t col = 0; col < (size_t)img.cols; ++col)
		{
			auto  pos   = cv::Point(col, row);
			auto& pixel = img.at<uchar>(pos);

			callback(pos, pixel);
		}
	}
}

inline void
iterate_4adj(cv::Mat& img, cv::Point pt, std::function<void(cv::Point&, uchar&)> callback, bool check_self = true)
{
	// boundary restrictions for 3x3 kernel
	size_t bound_row, bound_col, start_col, start_row, num_adj;

	bound_row = (pt.y >= (img.rows - 1)) ? img.rows : pt.y + 2;
	bound_col = (pt.x >= (img.cols - 1)) ? img.cols : pt.x + 2;
	start_col = pt.x - ((pt.x <= 0) ? 0 : 1);
	start_row = pt.y - ((pt.y <= 0) ? 0 : 1);

	// seek 8 adjacency 3x3 kernel grid
	for (size_t row = start_row; row < (size_t)bound_row; row++)
	{
		for (size_t col = start_col; col < (size_t)bound_col; col++)
		{

			auto  pos   = cv::Point(col, row);
			auto& pixel = img.at<uchar>(pos);

			// 4 adj check
			if (not (pos.x == pt.x || pos.y == pt.y || (check_self && pos == pt)))
				continue;

			callback(pos, pixel);
		}
	}
}

inline void
iterate_8adj(cv::Mat& img, cv::Point pt, std::function<void(cv::Point&, uchar&)> callback, bool check_self = true)
{
	// boundary restrictions for 3x3 kernel
	size_t bound_row, bound_col, start_col, start_row, num_adj;

	bound_row = (pt.y >= (img.rows - 1)) ? img.rows : pt.y + 2;
	bound_col = (pt.x >= (img.cols - 1)) ? img.cols : pt.x + 2;
	start_col = pt.x - ((pt.x <= 0) ? 0 : 1);
	start_row = pt.y - ((pt.y <= 0) ? 0 : 1);

	// seek 8 adjacency 3x3 kernel grid
	for (size_t row = start_row; row < (size_t)bound_row; row++)
	{
		for (size_t col = start_col; col < (size_t)bound_col; col++)
		{
			auto  pos   = cv::Point(col, row);
			auto& pixel = img.at<uchar>(pos);

			if (check_self || (not check_self && pos != pt))
				callback(pos, pixel);
		}
	}
}

template<typename D = std::chrono::microseconds, typename F> float
benchmark(F lambda, bool print = true)
{
	// start timing
	auto start = std::chrono::high_resolution_clock::now();

	// call function
	lambda();

	// stop timing
	auto end = std::chrono::high_resolution_clock::now();

	// resolve time/duration suffix
	auto dur_suffix = "[undefined]";

	if constexpr (std::is_same<D, std::chrono::nanoseconds>::value)
		dur_suffix = "ns";

	if constexpr (std::is_same<D, std::chrono::microseconds>::value)
		dur_suffix = "us";
	
	if constexpr (std::is_same<D, std::chrono::milliseconds>::value)
		dur_suffix = "ms";

	if constexpr (std::is_same<D, std::chrono::seconds>::value)
		dur_suffix = "s";
	
	// calculate duration
	auto dur = std::chrono::duration_cast<D>(end - start).count();

	// print result
	if (print)
		std::cout << "function call took: " << dur << " " << dur_suffix << std::endl;

	return dur;
}

template <typename T>
void
substract_vector(std::vector<T>& a, const std::vector<T>& b)
{
    typename std::vector<T>::iterator       it = a.begin();
    typename std::vector<T>::const_iterator it2 = b.begin();

    while (it != a.end())
    {
        while (it2 != b.end() && it != a.end())
        {
            if (*it == *it2)
            {
                it = a.erase(it);
                it2 = b.begin();
            }

            else
                ++it2;
        }
        if (it != a.end())
            ++it;

        it2 = b.begin();
    }
}