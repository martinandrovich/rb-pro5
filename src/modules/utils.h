#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <functional>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../constants.h"
#include "types/pose.h"

// -- morph structuring elements  -------------------------------------------------

namespace MORPH
{
	inline const auto STR_ELM_RECT_2x2 = cv::getStructuringElement(cv::MORPH_RECT,    cv::Size(2, 2));
	inline const auto STR_ELM_RECT_5x5 = cv::getStructuringElement(cv::MORPH_RECT,    cv::Size(5, 5));
	inline const auto STR_ELM_RECT_9x9 = cv::getStructuringElement(cv::MORPH_RECT,    cv::Size(9, 9));
	inline const auto STR_ELM_CROS_2x2 = cv::getStructuringElement(cv::MORPH_CROSS,   cv::Size(2, 2));
	inline const auto STR_ELM_ELPS_2x2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
}

// -- pixel  ----------------------------------------------------------------------

namespace PIXEL
{
	// pixel values
	constexpr auto BLACK   = 0;
	constexpr auto WHITE   = 255;
	constexpr auto GRAY    = 127;

	constexpr auto NOT_OBS = WHITE;
	constexpr auto IS_OBS  = BLACK;

	// cardinal directions
	inline const auto DIR_NW = cv::Point(-1, -1);
	inline const auto DIR_N  = cv::Point( 0, -1);
	inline const auto DIR_NE = cv::Point( 1, -1);
	inline const auto DIR_W  = cv::Point(-1,  0);
	inline const auto DIR_E  = cv::Point( 1,  0);
	inline const auto DIR_SW = cv::Point(-1,  1);
	inline const auto DIR_S  = cv::Point( 0,  1);
	inline const auto DIR_SE = cv::Point( 1,  1);

	// point locations
	template<typename T = uchar>
	inline auto TL(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_NW); }
	
	template<typename T = uchar>
	inline auto TM(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_N);  }

	template<typename T = uchar>
	inline auto TR(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_NE); }

	template<typename T = uchar>
	inline auto ML(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_W);  }

	template<typename T = uchar>
	inline auto MR(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_E);  }

	template<typename T = uchar>
	inline auto BL(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_SW); }

	template<typename T = uchar>
	inline auto BM(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_S);  }

	template<typename T = uchar>
	inline auto BR(const cv::Mat& img, const cv::Point& pt) { return img.at<T>(pt + DIR_SE); }

	// pixel class
	template<typename T = uchar>
	struct pixel_t
	{
		pixel_t(const cv::Mat& img, const cv::Point& pt) : img(img), pt(pt) {}

		auto tl() { return img.at<T>(pt + DIR_NW); }
		auto tm() { return img.at<T>(pt + DIR_N);  }
		auto tr() { return img.at<T>(pt + DIR_NE); }
		auto ml() { return img.at<T>(pt + DIR_W);  }
		auto mm() { return img.at<T>(pt);          }
		auto mr() { return img.at<T>(pt + DIR_E);  }
		auto bl() { return img.at<T>(pt + DIR_SW); }
		auto bm() { return img.at<T>(pt + DIR_S);  }
		auto br() { return img.at<T>(pt + DIR_SE); }

		cv::Mat img;
		cv::Point pt;
	};
}

// -- helper methods  -------------------------------------------------------------

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

inline bool
pt_within_boundary(const cv::Mat& img, const cv::Point& pt)
{
	return not ((pt.x - 1) < 0 || (pt.y - 1) < 0 || (pt.x + 1) > img.cols || (pt.y + 1) > img.rows);
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
iterate_rows(cv::Mat& img, std::function<void(cv::Point&, uchar&)> callback)
{
	iterate_mat(img, callback);
}

inline void
iterate_cols(cv::Mat& img, std::function<void(cv::Point&, uchar&)> callback)
{
	for (size_t col = 0; col < (size_t)img.cols; ++col)
	{
		for (size_t row = 0; row < (size_t)img.rows; ++row)
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

template<typename T>
inline void
iterate_3x3(cv::Mat& img, cv::Point pt, std::function<void(cv::Point&, T&)> callback)
{
	// boundary restrictions for 3x3 kernel
	size_t bound_row, bound_col, start_col, start_row;

	bound_row = (pt.y >= (img.rows - 1)) ? img.rows : pt.y + 2;
	bound_col = (pt.x >= (img.cols - 1)) ? img.cols : pt.x + 2;
	start_col = pt.x - ((pt.x <= 0) ? 0 : 1);
	start_row = pt.y - ((pt.y <= 0) ? 0 : 1);

	// seek 3x3 kernel grid
	for (size_t row = start_row; row < (size_t)bound_row; row++)
	{
		for (size_t col = start_col; col < (size_t)bound_col; col++)
		{
			auto  pos   = cv::Point(col, row);
			auto& pixel = img.at<T>(pos);

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

inline float
norm_pdf(float x, float m = 0, float s = 1)
{
	float a = (x - m) / s;
	return INV_SQRT_2PI / s * std::exp(-0.5f * a * a);
}

inline float
euclidean_dist(const cv::Point2f& a, const cv::Point2f& b)
{
	cv::Point diff = a - b;
	return diff.x * diff.x + diff.y * diff.y;
}

template<typename T>
inline void
add_border(cv::Mat& img, int thickness, T color, bool padded = false)
{
	if (padded)
	{
		cv::copyMakeBorder(img, img, thickness, thickness, thickness, thickness, cv::BORDER_CONSTANT, color);
	}
	else
	{
		cv::Rect border(cv::Point(0, 0), img.size());
		cv::rectangle(img, border, color, thickness);
	}
}

inline cv::Mat
thin_edges(const cv::Mat& img)
{

	using namespace cv;

	auto ThinSubiteration1 = [](Mat & pSrc, Mat & pDst)
	{
		int rows = pSrc.rows;
		int cols = pSrc.cols;
		pSrc.copyTo(pDst);
		for(int i = 0; i < rows; i++) {
				for(int j = 0; j < cols; j++) {
						if(pSrc.at<float>(i, j) == 1.0f) {
								/// get 8 neighbors
								/// calculate C(p)
								int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
								int neighbor1 = (int) pSrc.at<float>( i-1, j);
								int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
								int neighbor3 = (int) pSrc.at<float>( i, j+1);
								int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
								int neighbor5 = (int) pSrc.at<float>( i+1, j);
								int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
								int neighbor7 = (int) pSrc.at<float>( i, j-1);
								int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
												 int(~neighbor3 & ( neighbor4 | neighbor5)) +
												 int(~neighbor5 & ( neighbor6 | neighbor7)) +
												 int(~neighbor7 & ( neighbor0 | neighbor1));
								if(C == 1) {
										/// calculate N
										int N1 = int(neighbor0 | neighbor1) +
														 int(neighbor2 | neighbor3) +
														 int(neighbor4 | neighbor5) +
														 int(neighbor6 | neighbor7);
										int N2 = int(neighbor1 | neighbor2) +
														 int(neighbor3 | neighbor4) +
														 int(neighbor5 | neighbor6) +
														 int(neighbor7 | neighbor0);
										int N = min(N1,N2);
										if ((N == 2) || (N == 3)) {
												/// calculate criteria 3
												int c3 = ( neighbor1 | neighbor2 | ~neighbor4) & neighbor3;
												if(c3 == 0) {
														pDst.at<float>( i, j) = 0.0f;
												}
										}
								}
						}
				}
		}
	};

	auto ThinSubiteration2 = [](Mat & pSrc, Mat & pDst)
	{
		int rows = pSrc.rows;
		int cols = pSrc.cols;
		pSrc.copyTo( pDst);
		for(int i = 0; i < rows; i++) {
				for(int j = 0; j < cols; j++) {
						if (pSrc.at<float>( i, j) == 1.0f) {
								/// get 8 neighbors
								/// calculate C(p)
							int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
							int neighbor1 = (int) pSrc.at<float>( i-1, j);
							int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
							int neighbor3 = (int) pSrc.at<float>( i, j+1);
							int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
							int neighbor5 = (int) pSrc.at<float>( i+1, j);
							int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
							int neighbor7 = (int) pSrc.at<float>( i, j-1);
								int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
										int(~neighbor3 & ( neighbor4 | neighbor5)) +
										int(~neighbor5 & ( neighbor6 | neighbor7)) +
										int(~neighbor7 & ( neighbor0 | neighbor1));
								if(C == 1) {
										/// calculate N
										int N1 = int(neighbor0 | neighbor1) +
												int(neighbor2 | neighbor3) +
												int(neighbor4 | neighbor5) +
												int(neighbor6 | neighbor7);
										int N2 = int(neighbor1 | neighbor2) +
												int(neighbor3 | neighbor4) +
												int(neighbor5 | neighbor6) +
												int(neighbor7 | neighbor0);
										int N = min(N1,N2);
										if((N == 2) || (N == 3)) {
												int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7;
												if(E == 0) {
														pDst.at<float>(i, j) = 0.0f;
												}
										}
								}
						}
				}
		}
	};

		bool bDone = false;
		int rows = img.rows;
		int cols = img.cols;

		auto inputarray  = img.clone();
		auto outputarray = img.clone();

		cv::bitwise_not(inputarray, inputarray);

		inputarray.convertTo(inputarray,CV_32FC1);

		inputarray.copyTo(outputarray);

		outputarray.convertTo(outputarray,CV_32FC1);

		/// pad source
		Mat p_enlarged_src = Mat(rows + 2, cols + 2, CV_32FC1);
		for(int i = 0; i < (rows+2); i++) {
			p_enlarged_src.at<float>(i, 0) = 0.0f;
			p_enlarged_src.at<float>( i, cols+1) = 0.0f;
		}
		for(int j = 0; j < (cols+2); j++) {
				p_enlarged_src.at<float>(0, j) = 0.0f;
				p_enlarged_src.at<float>(rows+1, j) = 0.0f;
		}
		for(int i = 0; i < rows; i++) {
				for(int j = 0; j < cols; j++) {
						if (inputarray.at<float>(i, j) >= 20.0f) {
								p_enlarged_src.at<float>( i+1, j+1) = 1.0f;
						}
						else
								p_enlarged_src.at<float>( i+1, j+1) = 0.0f;
				}
		}

		/// start to thin
		Mat p_thinMat1 = Mat::zeros(rows + 2, cols + 2, CV_32FC1);
		Mat p_thinMat2 = Mat::zeros(rows + 2, cols + 2, CV_32FC1);
		Mat p_cmp = Mat::zeros(rows + 2, cols + 2, CV_8UC1);

		while (bDone != true) {
				/// sub-iteration 1
				ThinSubiteration1(p_enlarged_src, p_thinMat1);
				/// sub-iteration 2
				ThinSubiteration2(p_thinMat1, p_thinMat2);
				/// compare
				compare(p_enlarged_src, p_thinMat2, p_cmp, CV_CMP_EQ);
				/// check
				int num_non_zero = countNonZero(p_cmp);
				if(num_non_zero == (rows + 2) * (cols + 2)) {
						bDone = true;
				}
				/// copy
				p_thinMat2.copyTo(p_enlarged_src);
		}
		// copy result
		for(int i = 0; i < rows; i++) {
				for(int j = 0; j < cols; j++) {
						outputarray.at<float>( i, j) = p_enlarged_src.at<float>( i+1, j+1);
				}
		}

		cv::Mat img_out;
		cv::convertScaleAbs(outputarray, img_out);
		cv::threshold(img_out, img_out, 0, 255, cv::THRESH_BINARY);
		cv::bitwise_not(img_out, img_out);

		return img_out;
}