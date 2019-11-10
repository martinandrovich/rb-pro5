#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../constants.h"
#include "types/pose.h"

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
scale_image(cv::Mat& img, const dim_t& dim_world, const float meter_per_px)
{
	// compute scaling factors
	auto scale_width  = dim_world.width  / (meter_per_px * img.cols);
	auto scale_height = dim_world.height / (meter_per_px * img.rows);
	auto scale        = (scale_width + scale_height) / 2.f;

	cv::resize(img, img, cv::Size(), scale_width, scale_height, cv::INTER_NEAREST);

	return scale;
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

inline cv::Mat
brushfire()
{
	// load image
	auto img = load_img(PATH_IMG_GVD_MAP, cv::IMREAD_GRAYSCALE);
	cv::resize(img, img, cv::Size(), 20.f, 20.f, cv::INTER_NEAREST);
	show_img("brushfire", img);

	// remove compression (e.g. if JPEG)
	img = img == 255;
	
	// invert and binarize
	img = (255 - img);
	img = cv::min(img, 1);

	// count zero-value pixels
	auto num_zero_px = img.rows * img.cols - cv::countNonZero(img);

	// 8-adjacency check for a given point, tries to find specified value within neighboors
	auto eight_adj_find = [&](const cv::Point& pt, size_t val)
	{
		// absolute bounds and starting position for seeking pixel
		// if seeking pixel starts in a corner or near image edge
		
		size_t bound_row, bound_col, start_col, start_row;

		bound_row = (pt.y >= (img.rows - 1)) ? img.rows : pt.y + 2;
		bound_col = (pt.x >= (img.cols - 1)) ? img.cols : pt.x + 2;
		start_col = pt.x - ((pt.x <= 0) ? 0 : 1);
		start_row = pt.y - ((pt.y <= 0) ? 0 : 1);

		// seek the kernel grid; return true if val is found
		for(size_t row = start_row; row < (size_t)bound_row; row++)
		{
			for(size_t col = start_col; col < (size_t)bound_col; col++)
			{
				if (img.at<uchar>({col, row}) == (uchar)val)
					return true;
			}
		}

		return false;
	};

	auto voronoi_check = [&](const cv::Point& pt)
	{
		
		// boundary check
		if ((pt.x - 1) < 0 || (pt.y - 1) < 0 || (pt.x + 1 > img.cols) || (pt.y + 1 > img.rows))
			return false;

		const auto& mm = img.at<uchar>({pt.x    , pt.y    });
		const auto& tl = img.at<uchar>({pt.x - 1, pt.y - 1});
		const auto& br = img.at<uchar>({pt.x + 1, pt.y + 1});
		const auto& tm = img.at<uchar>({pt.x    , pt.y - 1});
		const auto& bm = img.at<uchar>({pt.x    , pt.y + 1});
		const auto& tr = img.at<uchar>({pt.x + 1, pt.y - 1});
		const auto& bl = img.at<uchar>({pt.x - 1, pt.y + 1});
		const auto& mr = img.at<uchar>({pt.x - 1, pt.y    });
		const auto& ml = img.at<uchar>({pt.x + 1, pt.y    });

		return
		(
			(tl == br && tm == bm && tr == bl && mr == ml) ||
			(tm == bm && mr == ml) ||
			(tl == br && tl != mm && br != mm) || // TL and BR diag
			(tm == bm && tm != mm && bm != mm) || // TM and BM vert
			(tr == bl && tr != mm && bl != mm) || // TR and BL diag
			(mr == ml && mr != mm && ml != mm)    // MR and ML horz
		);
	};

	// brushfire algorithm
	// iterate all 0-valued pixels and increment n each (complete) iteration
	// if zero-th pixel has the value n within its adjacency, then set that pixel to n + 1
	for (size_t n = 1; num_zero_px != 0 ; ++n)
	{

		if (n > 255)
			throw std::runtime_error("Value of n has exceeded 255.");

		for (size_t row = 0; row < (size_t)img.rows; ++row)
		{
			for (size_t col = 0; col < (size_t)img.cols; ++col)
			{
				auto  pos   = cv::Point(col, row);
				auto& pixel = img.at<uchar>(pos);

				if (pixel == 0 && eight_adj_find(pos, n))
				{
					 pixel = n + 1;
					--num_zero_px;
				}

			}
		}
	}

	for (size_t row = 0; row < (size_t)img.rows; ++row)
	{
		for (size_t col = 0; col < (size_t)img.cols; ++col)
		{
			auto  pos   = cv::Point(col, row);
			auto& pixel = img.at<uchar>(pos);

			if (pixel != 1 && voronoi_check(pos))
				pixel = 255;
		}
	}

	show_img("brushfire", img);
	return img;
}