#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <stdexcept>
#include <optional>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../constants.h"
#include "utils.h"

// --------------------------------------------------------------------------------
// declarations of helper structures for ::geometry
// --------------------------------------------------------------------------------

using box_t    = std::array<cv::Point, 2>;
using vertex_t = cv::Point;

struct line_t
{
	cv::Point from;
	cv::Point to;

	box_t
	bounding_box() const;
};

struct gvd_t
{
	cv::Mat img;
};

// --------------------------------------------------------------------------------
// declarations for ::geometry
// --------------------------------------------------------------------------------

namespace geometry
{
	
	// -- brushfire & GVD -------------------------------------------------------------

	std::vector<cv::Vec4i>
	extract_lines(const cv::Mat& img);

	std::vector<vertex_t>
	extract_vertices(const cv::Mat& img);

	cv::Mat
	draw_vertices(const cv::Mat& img, std::vector<vertex_t> vertices);

	cv::Mat
	brushfire(const cv::Mat& img);

	gvd_t
	gvd(const cv::Mat& img);

	// -- line segment intersection ---------------------------------------------------

	// https://martin-thoma.com/how-to-check-if-two-line-segments-intersect/

	float
	cross(const cv::Point& a, const cv::Point& b);

	bool
	bounding_box_intersect(const box_t& a, const box_t& b);

	bool
	is_pt_on_line(const line_t& line, const cv::Point& pt);

	bool
	is_pt_right_of_line(const line_t& line, const cv::Point& pt);

	bool
	line_touch_or_cross(const line_t& a, const line_t& b);

	bool
	segment_intersect(const line_t& a, const line_t& b);

	bool
	segment_intersect(const cv::Point& A, const cv::Point& B, const cv::Point& C, const cv::Point& D);

	bool
	segment_intersect_safe(const line_t& a, const line_t& b);

	std::optional<cv::Point>
	segment_intersect_at(const line_t& a, const line_t& b);

	// -- test methods ----------------------------------------------------------------

	void
	test_brushfire_and_gvd();

	void
	test_segment_intersect();
}

// --------------------------------------------------------------------------------
// definitions for ::geometry
// --------------------------------------------------------------------------------

// -- brushfire & GVD -------------------------------------------------------------

inline std::vector<vertex_t>
geometry::extract_vertices(const cv::Mat& img)
{
	// https://stackoverflow.com/questions/33646643/store-details-of-a-binary-image-consisting-simple-polygons
	
	// must be grayscale
	if (img.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// vector of contours
	std::vector<std::vector<cv::Point>> vec_contours;
	cv::findContours(img, vec_contours, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1);

	// flatten vector of contours into vector of vertices
	std::vector<vertex_t> vec_vertices;
	for (const auto& vec : vec_contours)
		vec_vertices.insert(vec_vertices.end(), vec.begin(), vec.end());

	return vec_vertices;
}

inline cv::Mat
geometry::draw_vertices(const cv::Mat& img, std::vector<vertex_t> vertices)
{
	cv::Mat img_vertices;
	cv::cvtColor(img, img_vertices, CV_GRAY2BGR);

	for (const auto& v : vertices)
		cv::circle(img_vertices, cv::Point(v.x, v.y), 3, cv::Scalar(255, 0, 255));

	return img_vertices;
}

inline cv::Mat
geometry::brushfire(const cv::Mat& img_map)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img, img_copy, img_map_inv, img_laplace, img_gvd;

	// clone
	img = img_map.clone();

	// remove compression (e.g. if JPEG)
	img = img == 255;

	// compute the inverse
	cv::bitwise_not(img, img_map_inv);
	
	// binarize the inverse
	img = cv::min(img_map_inv, 1);

	// count zero-value pixels
	auto num_zero_px = img.rows * img.cols - cv::countNonZero(img);

	// 8-adjacency check for a given point, tries to find specified value within neighboors
	auto eight_adj_find = [&](const cv::Point& pt, size_t val)
	{

		// absolute bounds and starting position for seeking pixel
		// if seeking pixel starts in a corner or near image edge
		
		size_t bound_row, bound_col, start_col, start_row, num_adj;

		bound_row = (pt.y >= (img.rows - 1)) ? img.rows : pt.y + 2;
		bound_col = (pt.x >= (img.cols - 1)) ? img.cols : pt.x + 2;
		start_col = pt.x - ((pt.x <= 0) ? 0 : 1);
		start_row = pt.y - ((pt.y <= 0) ? 0 : 1);
		num_adj   = 0;

		// seek the kernel grid; return true if val is found
		for(size_t row = start_row; row < (size_t)bound_row; row++)
		{
			for(size_t col = start_col; col < (size_t)bound_col; col++)
			{
				if (img.at<uchar>(cv::Point(col, row)) == (uchar)val)
					num_adj++;
			}
		}

		return num_adj;
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

				if (auto num_adj = eight_adj_find(pos, n); pixel == 0 && num_adj > 0)
				{
					pixel = n + 1;
					--num_zero_px;
				}

			}
		}
	}

	return img;
}

inline gvd_t
geometry::gvd(const cv::Mat& img_map)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_map_inv, img_bf, img_laplace, img_gvd, img_gvd_dil;

	// compute inverse
	cv::bitwise_not(img_map, img_map_inv);

	// compute brushfure
	img_bf = brushfire(img_map);

	// use laplace operator to find gradient, i.e. the GVD
	cv::Laplacian(img_bf, img_laplace, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(img_laplace, img_gvd);
	
	// amplify all non-zero pixels
	img_gvd *= 255;

	// remove obstacle outlinees from GVD (subtract the inverse)
	img_gvd -= img_map_inv;

	// invert the GVD, making the edges black
	cv::bitwise_not(img_gvd, img_gvd);

	// perform closing
	;
	
	// dilute the GVD (thin the black edges)
	size_t dil_sz  = 1;
	auto   str_elm = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dil_sz + 1, dil_sz + 1));

	cv::dilate(img_gvd, img_gvd_dil, str_elm);
	//show_img("gvd (dilated)", img_gvd_dil);

	// needs more work
	// an alternative appraoch is to detect the vertices
	// and draw lines manually

	return gvd_t{img_gvd};
}

// -- line segment intersection ---------------------------------------------------

inline box_t
line_t::bounding_box() const
{
	std::array<cv::Point, 2> box;

	box[0] = cv::Point(std::min(this->from.x, this->to.x), std::min(this->from.y, this->to.y));
	box[1] = cv::Point(std::max(this->from.x, this->to.x), std::max(this->from.y, this->to.y));

	return box;
}

inline float
geometry::cross(const cv::Point& a, const cv::Point& b)
{
	return a.x * b.y - b.x * a.y;
}

inline bool
geometry::bounding_box_intersect(const box_t& a, const box_t& b)
{
	return a[0].x <= b[1].x && a[1].x >= b[0].x && a[0].y <= b[1].y	&& a[1].y >= b[0].y;
}

inline bool
geometry::is_pt_on_line(const line_t& line, const cv::Point& pt)
{
	auto line_new = line_t{ { 0, 0 }, cv::Point(line.to.x - line.from.x, line.to.y - line.from.y) };	
	auto pt_new   = cv::Point(pt.x - line.from.x, pt.y - line.from.y);

	return (std::abs(cross(line_new.to, pt_new))< EPSILON);
}

inline bool
geometry::is_pt_right_of_line(const line_t& line, const cv::Point& pt)
{
	auto line_temp = line_t{ { 0, 0 }, cv::Point(line.to.x - line.from.x, line.to.y - line.from.y) };	
	auto pt_temp   = cv::Point(pt.x - line.from.x, pt.y - line.from.y);

	return (cross(line_temp.to, pt_temp) < 0);
}

inline bool
geometry::line_touch_or_cross(const line_t& a, const line_t& b)
{
	return (is_pt_on_line(a, b.from) || is_pt_on_line(a, b.to) || (is_pt_right_of_line(a, b.from) ^ is_pt_right_of_line(a, b.to)));
}

inline bool
geometry::segment_intersect_safe(const line_t& a, const line_t& b)
{
	auto box_a = a.bounding_box();
	auto box_b = b.bounding_box();

	return bounding_box_intersect(box_a, box_b) && line_touch_or_cross(a, b) && line_touch_or_cross(b, a);
}

inline std::optional<cv::Point>
geometry::segment_intersect_at(const line_t& a, const line_t& b)
{

	// usage
	// if (auto pt = segment_intersect_at(A, B)) {...}

	if (not segment_intersect_safe(a, b))
		return std::nullopt;

	else
		throw std::runtime_error(ERR_NO_IMPL);
		// return cv::Point(0,0);
}

inline bool
geometry::segment_intersect(const line_t& a, const line_t& b)
{
	auto& A = a.from;
	auto& B = a.to;
	auto& C = b.from;
	auto& D = b.to;

	return segment_intersect(A, B, C, D);
}

inline bool
geometry::segment_intersect(const cv::Point& A, const cv::Point& B, const cv::Point& C, const cv::Point& D)
{
	auto ccw = [](const cv::Point& A, const cv::Point& B, const cv::Point& C)
	{
		return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x);
	};

	return ((ccw(A,C,D) != ccw(B,C,D)) && (ccw(A,B,C) != ccw(A,B,D)));
}

// -- test methods ----------------------------------------------------------------

inline void
geometry::test_brushfire_and_gvd()
{
	// load and scale image
	auto img = load_img(PATH_IMG_GVD_MAP, cv::IMREAD_GRAYSCALE);
	cv::resize(img, img, cv::Size(), 20.f, 20.f, cv::INTER_NEAREST);

	// show input
	show_img("map", img);
	
	// brushfire
	auto img_bf = brushfire(img);
	show_img("brushfire", img_bf);

	// gvd
	auto img_gvd = gvd(img).img;
	show_img("gvd", img_gvd);

	// vertices
	auto vert = extract_vertices(img_gvd);
	auto img_vert = draw_vertices(img_gvd, vert);
	show_img("vertices", img_vert);
}

inline void
geometry::test_segment_intersect()
{
	line_t A = { cv::Point(0,0), cv::Point(3,3) };
	line_t B = { cv::Point(0,3), cv::Point(3,0) };

	bool intersect = false;
	bool intersect_v2 = false;

	benchmark<std::chrono::nanoseconds>([&]
	{
		intersect = segment_intersect_safe(A, B);
	});

	benchmark<std::chrono::nanoseconds>([&]
	{
		intersect_v2 = segment_intersect(A.from, A.to, B.from, B.to);
	});

	std::cout << "line intersect: "      << std::boolalpha << intersect << std::endl;
	std::cout << "line intersect (v2): " << std::boolalpha << intersect_v2 << std::endl;
}