#pragma once

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <functional>

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

	// patterns

	inline const cv::Mat PAT_FILL              = (cv::Mat_<uchar>(3,3) << BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK);
	inline const cv::Mat PAT_EMPTY             = (cv::Mat_<uchar>(3,3) << WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE);
	inline const cv::Mat PAT_CROSS             = (cv::Mat_<uchar>(3,3) << WHITE, BLACK, WHITE, BLACK, BLACK, BLACK, WHITE, BLACK, WHITE);

	// V-shapes
	inline const cv::Mat PAT_V_UP              = (cv::Mat_<uchar>(3,3) << WHITE, BLACK, WHITE, WHITE, BLACK, WHITE, BLACK, WHITE, BLACK);
	inline const cv::Mat PAT_V_DOWN            = (cv::Mat_<uchar>(3,3) << BLACK, WHITE, BLACK, WHITE, BLACK, WHITE, WHITE, BLACK, WHITE);
	inline const cv::Mat PAT_V_LEFT            = (cv::Mat_<uchar>(3,3) << WHITE, WHITE, BLACK, BLACK, BLACK, WHITE, WHITE, WHITE, BLACK);
	inline const cv::Mat PAT_V_RIGHT           = (cv::Mat_<uchar>(3,3) << BLACK, WHITE, WHITE, WHITE, BLACK, BLACK, BLACK, WHITE, WHITE);
	inline const auto    PAT_V_VEC             = std::array{ PAT_V_UP, PAT_V_DOWN, PAT_V_LEFT, PAT_V_RIGHT };

	// T-shapes
	inline const cv::Mat PAT_T_UP              = (cv::Mat_<uchar>(3,3) << WHITE, BLACK, WHITE, BLACK, BLACK, BLACK, WHITE, WHITE, WHITE);
	inline const cv::Mat PAT_T_DOWN            = (cv::Mat_<uchar>(3,3) << WHITE, WHITE, WHITE, BLACK, BLACK, BLACK, WHITE, BLACK, WHITE);
	inline const cv::Mat PAT_T_LEFT            = (cv::Mat_<uchar>(3,3) << WHITE, BLACK, WHITE, BLACK, BLACK, WHITE, WHITE, BLACK, WHITE);
	inline const cv::Mat PAT_T_RIGHT           = (cv::Mat_<uchar>(3,3) << WHITE, BLACK, WHITE, WHITE, BLACK, BLACK, WHITE, BLACK, WHITE);
	inline const auto    PAT_T_VEC             = std::array{ PAT_T_UP, PAT_T_DOWN, PAT_T_LEFT, PAT_T_RIGHT };
	
	// asymmetric V-shapes
	inline const cv::Mat PAT_ASYM_V_UP_LEFT    = (cv::Mat_<uchar>(3,3) << WHITE, BLACK, WHITE, BLACK, BLACK, WHITE, WHITE, WHITE, BLACK);
	inline const cv::Mat PAT_ASYM_V_UP_RIGHT   = (cv::Mat_<uchar>(3,3) << WHITE, BLACK, WHITE, WHITE, BLACK, BLACK, BLACK, WHITE, WHITE);
	inline const cv::Mat PAT_ASYM_V_DOWN_LEFT  = (cv::Mat_<uchar>(3,3) << WHITE, WHITE, BLACK, BLACK, BLACK, WHITE, WHITE, BLACK, WHITE);
	inline const cv::Mat PAT_ASYM_V_DOWN_RIGHT = (cv::Mat_<uchar>(3,3) << BLACK, WHITE, WHITE, WHITE, BLACK, BLACK, WHITE, BLACK, WHITE);
	inline const auto    PAT_ASYM_V_VEC        = std::array{ PAT_ASYM_V_UP_LEFT, PAT_ASYM_V_UP_RIGHT, PAT_ASYM_V_DOWN_LEFT, PAT_ASYM_V_DOWN_RIGHT };

	// diagonals
	inline const cv::Mat PAT_DIAG_NW           = (cv::Mat_<uchar>(3,3) << BLACK, WHITE, WHITE, WHITE, BLACK, WHITE, WHITE, WHITE, WHITE);
	inline const cv::Mat PAT_DIAG_NE           = (cv::Mat_<uchar>(3,3) << WHITE, WHITE, BLACK, WHITE, BLACK, WHITE, WHITE, WHITE, WHITE);
	inline const cv::Mat PAT_DIAG_SW           = (cv::Mat_<uchar>(3,3) << WHITE, WHITE, WHITE, WHITE, BLACK, WHITE, BLACK, WHITE, WHITE);
	inline const cv::Mat PAT_DIAG_SE           = (cv::Mat_<uchar>(3,3) << WHITE, WHITE, WHITE, WHITE, BLACK, WHITE, WHITE, WHITE, BLACK);
	inline const auto    PAT_DIAG_VEC          = std::array{ PAT_DIAG_NW, PAT_DIAG_NE, PAT_DIAG_SW, PAT_DIAG_SE };

	// cardinal directionscv::
	inline const auto DIR_NONE                 = cv::Point( 0,  0);
	inline const auto DIR_NW                   = cv::Point(-1, -1);
	inline const auto DIR_N                    = cv::Point( 0, -1);
	inline const auto DIR_NE                   = cv::Point( 1, -1);
	inline const auto DIR_W                    = cv::Point(-1,  0);
	inline const auto DIR_E                    = cv::Point( 1,  0);
	inline const auto DIR_SW                   = cv::Point(-1,  1);
	inline const auto DIR_S                    = cv::Point( 0,  1);
	inline const auto DIR_SE                   = cv::Point( 1,  1);
	inline const auto DIR_VEC                  = std::array{ DIR_NW, DIR_N, DIR_NE, DIR_W, DIR_E, DIR_SW, DIR_S, DIR_SE };

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

// -- helper constructs  ----------------------------------------------------------

// blob type for connected components
struct blob_t
{
	std::vector<cv::Point> vec_pts;
	size_t size;
};

// -- helper methods  -------------------------------------------------------------

inline bool
file_exists(const std::string& filename)
{
	std::ifstream f(filename.c_str());
	return f.good();
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
combine_img(cv::Mat& img_color, const cv::Mat& img_gray)
{
	std::vector<cv::Mat> channels;
	cv::split(img_color, channels);

	for (auto& c : channels)
		cv::bitwise_and(c, img_gray, c);

	cv::merge(channels, img_color);
}

inline bool
pt_within_boundary(const cv::Mat& img, const cv::Point& pt)
{
	return not ((pt.x - 1) < 0 || (pt.y - 1) < 0 || (pt.x + 1) > img.cols || (pt.y + 1) > img.rows);
}

inline bool
kernel_3x3_within_boundary(const cv::Mat& img, const cv::Point& pt)
{
	return not ((pt.x - 2) < 0 || (pt.y - 2) < 0 || (pt.x + 2) > img.cols || (pt.y + 2) > img.rows);
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

template<typename T = uchar>
inline bool
mat_equal(const cv::Mat& a, const cv::Mat& b)
{
	return std::equal(a.begin<T>(), a.end<T>(), b.begin<T>());

	// cv::Mat result;
	// cv::bitwise_xor(a, b, result);
	// return (cv::countNonZero(result) == 0);

	// cv::Mat diff = a != b;
	// return (cv::countNonZero(diff) == 0);
}

inline bool
match_pattern_3x3(const cv::Mat& img, const cv::Point& pt_center, const cv::Mat& pattern)
{
	constexpr auto ROI_SIZE = 3;

	if (pattern.rows != ROI_SIZE || pattern.cols != ROI_SIZE)
		throw std::runtime_error(ERR_PATTERN_MISMATCH_3x3);

	if (not kernel_3x3_within_boundary(img, pt_center))
		return false;

	auto roi = img(cv::Rect(pt_center.x - 1, pt_center.y - 1, ROI_SIZE, ROI_SIZE));

	return mat_equal(roi, pattern);
}

inline bool
match_pattern_3x3(const cv::Mat& img, const cv::Point& pt_center, std::array<uchar, 9> pattern)
{
	auto mat_pattern = cv::Mat(3, 3, CV_8U, pattern.data());
	return match_pattern_3x3(img, pt_center, mat_pattern);
}

inline std::vector<blob_t>
connected_blobs(const cv::Mat& img, int connectivity = 8, bool sort_by_size = false)
{
	// assert grayscale image
	if (img.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_inv, img_out, labels, stats, centroids;

	// invert image; black pixels are treated as a background
	cv::bitwise_not(img, img_inv);

	// segregate image into blobs
	auto n_labels = cv::connectedComponentsWithStats(img_inv, labels, stats, centroids, connectivity);

	// create map of blobs
	std::vector<blob_t> vec_blobs(n_labels);
	for (int r = 0; r < img.rows; ++r)
	{
		for (int c = 0; c < img.cols; ++c)
		{
			auto label = labels.at<int>(r, c);
			vec_blobs[label].vec_pts.emplace_back(cv::Point(c, r));
		}
	}

	// determine sizes of each blob; index 1 is background
	for (size_t label = 1; label < n_labels; label++)
		vec_blobs[label].size = stats.at<int>(label, cv::CC_STAT_AREA);

	// remove background blob, being the first
	vec_blobs.erase(vec_blobs.begin() + 0);

	// sort in descending order; largest first
	if (sort_by_size)
	{
		std::sort(vec_blobs.begin(), vec_blobs.end(), [](const auto& a, const auto& b) {
			return a.size > b.size;
		});
	}

	return vec_blobs;
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

	// https://answers.opencv.org/question/3207/what-is-a-good-thinning-algorithm-for-getting-the-skeleton-of-characters-for-ocr/
	// https://stackoverflow.com/questions/6409759/extracting-segments-from-a-list-of-8-connected-pixels

	auto thin_subiter1 = [](const cv::Mat& mat_src, cv::Mat& mat_dst)
	{
		int rows = mat_src.rows;
		int cols = mat_src.cols;
		mat_src.copyTo(mat_dst);

		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				if (mat_src.at<float>(i, j) == 1.0f)
				{
					// get 8 neighbors
					// calculate C(p)
					int neighbor0 = (int) mat_src.at<float>(i-1, j-1);
					int neighbor1 = (int) mat_src.at<float>(i-1, j);
					int neighbor2 = (int) mat_src.at<float>(i-1, j+1);
					int neighbor3 = (int) mat_src.at<float>(i,   j+1);
					int neighbor4 = (int) mat_src.at<float>(i+1, j+1);
					int neighbor5 = (int) mat_src.at<float>(i+1, j);
					int neighbor6 = (int) mat_src.at<float>(i+1, j-1);
					int neighbor7 = (int) mat_src.at<float>(i,   j-1);
					
					int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
							int(~neighbor3 & ( neighbor4 | neighbor5)) +
							int(~neighbor5 & ( neighbor6 | neighbor7)) +
							int(~neighbor7 & ( neighbor0 | neighbor1));

					if(C == 1)
					{
						// calculate N
						int N1 = int(neighbor0 | neighbor1) +
								 int(neighbor2 | neighbor3) +
								 int(neighbor4 | neighbor5) +
								 int(neighbor6 | neighbor7);

						int N2 = int(neighbor1 | neighbor2) +
								 int(neighbor3 | neighbor4) +
								 int(neighbor5 | neighbor6) +
								 int(neighbor7 | neighbor0);

						int N = std::min(N1, N2);

						if ((N == 2) || (N == 3))
						{
							// calculate criteria 3
							int c3 = (neighbor1 | neighbor2 | ~neighbor4) & neighbor3;
							
							if (c3 == 0)
								mat_dst.at<float>(i, j) = 0.0f;
						}
					}
				}
			}
		}
	};

	auto thin_subiter2 = [](const cv::Mat& mat_src, cv::Mat& mat_dst)
	{
		int rows = mat_src.rows;
		int cols = mat_src.cols;
		mat_src.copyTo( mat_dst);

		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				if (mat_src.at<float>( i, j) == 1.0f)
				{
					// get 8 neighbors
					// calculate C(p)
					int neighbor0 = (int) mat_src.at<float>(i-1, j-1);
					int neighbor1 = (int) mat_src.at<float>(i-1, j);
					int neighbor2 = (int) mat_src.at<float>(i-1, j+1);
					int neighbor3 = (int) mat_src.at<float>(i,   j+1);
					int neighbor4 = (int) mat_src.at<float>(i+1, j+1);
					int neighbor5 = (int) mat_src.at<float>(i+1, j);
					int neighbor6 = (int) mat_src.at<float>(i+1, j-1);
					int neighbor7 = (int) mat_src.at<float>(i,   j-1);

					int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
							int(~neighbor3 & ( neighbor4 | neighbor5)) +
							int(~neighbor5 & ( neighbor6 | neighbor7)) +
							int(~neighbor7 & ( neighbor0 | neighbor1));

					if(C == 1)
					{
						// calculate N
						int N1 = int(neighbor0 | neighbor1) +
								 int(neighbor2 | neighbor3) +
								 int(neighbor4 | neighbor5) +
								 int(neighbor6 | neighbor7);

						int N2 = int(neighbor1 | neighbor2) +
								 int(neighbor3 | neighbor4) +
								 int(neighbor5 | neighbor6) +
								 int(neighbor7 | neighbor0);

						int N = std::min(N1, N2);

						if((N == 2) || (N == 3))
						{
							int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7;

							if(E == 0)
								mat_dst.at<float>(i, j) = 0.0f;
						}
					}
				}
			}
		}
	};

	bool done = false;
	int  rows = img.rows;
	int  cols = img.cols;

	auto mat_input  = img.clone();
	auto mat_output = img.clone();

	// invert input (lines to thin must be white)
	cv::bitwise_not(mat_input, mat_input);

	// convert data
	mat_input.convertTo(mat_input, CV_32FC1);
	mat_input.copyTo(mat_output);
	mat_output.convertTo(mat_output, CV_32FC1);

	// pad source
	cv::Mat mat_src_enlarged = cv::Mat(rows + 2, cols + 2, CV_32FC1);

	for (int i = 0; i < (rows+2); i++)
	{
		mat_src_enlarged.at<float>(i, 0) = 0.0f;
		mat_src_enlarged.at<float>(i, cols+1) = 0.0f;
	}

	for (int j = 0; j < (cols+2); j++)
	{
			mat_src_enlarged.at<float>(0, j) = 0.0f;
			mat_src_enlarged.at<float>(rows+1, j) = 0.0f;
	}

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (mat_input.at<float>(i, j) >= 20.0f)
					mat_src_enlarged.at<float>(i+1, j+1) = 1.0f;
			else
				mat_src_enlarged.at<float>(i+1, j+1) = 0.0f;
		}
	}

	// start thinning process
	cv::Mat mat_thin1 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
	cv::Mat mat_thin2 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
	cv::Mat mat_cmp   = cv::Mat::zeros(rows + 2, cols + 2, CV_8UC1);

	while (not done)
	{
		// sub-iteration 1
		thin_subiter1(mat_src_enlarged, mat_thin1);

		// sub-iteration 2
		thin_subiter2(mat_thin1, mat_thin2);

		// compare
		cv::compare(mat_src_enlarged, mat_thin2, mat_cmp, cv::CMP_EQ);

		// check
		if (cv::countNonZero(mat_cmp) == (rows + 2) * (cols + 2))
			done = true;

		// copy
		mat_thin2.copyTo(mat_src_enlarged);
	}

	// copy result
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			mat_output.at<float>(i, j) = mat_src_enlarged.at<float>(i+1, j+1);
		}
	}

	// convert result to 8-bit
	cv::Mat img_out;
	cv::convertScaleAbs(mat_output, img_out);
	cv::threshold(img_out, img_out, 0, 255, cv::THRESH_BINARY);
	cv::bitwise_not(img_out, img_out);

	return img_out;
}