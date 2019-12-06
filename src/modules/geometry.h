#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <stdexcept>
#include <optional>
#include <csignal>
#include <algorithm>

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

// --------------------------------------------------------------------------------
// declarations for ::geometry
// --------------------------------------------------------------------------------

namespace geometry
{

	// -- points ----------------------------------------------------------------------

	cv::Mat
	draw_pt(const cv::Mat& img, cv::Point pt, cv::Scalar color = cv::Scalar(255, 0, 255));

	cv::Mat
	draw_pts(const cv::Mat& img, std::vector<cv::Point> pts, cv::Scalar color = cv::Scalar(255, 0, 255));
	
	cv::Point
	avg_pt(const std::vector<cv::Point>& pts);
	
	std::optional<std::vector<cv::Point>>
	pts_within_radius(std::vector<cv::Point>& pts, const cv::Point& center, float radius);

	bool
	pts_connected(const cv::Mat& img, const cv::Point& a,  const cv::Point& b, float max_deviation = 0);

	std::vector<cv::Vec4i>
	extract_lines(const cv::Mat& img);

	std::vector<vertex_t>
	extract_vertices(const cv::Mat& img, bool remove_duplicates = false, float duplicates_radius = 2.f);

	// -- line segment intersection ---------------------------------------------------

	// https://martin-thoma.com/how-to-check-if-two-line-segments-intersect/

	size_t
	count_duplicate_lines(std::vector<line_t> vec_lines);

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
	test_gvd_draw_edges();
	
	void
	test_gvd_houghlines();

	void
	test_gvd_houghlinesp();
	
	void
	test_brushfire_and_gvd();

	void
	test_segment_intersect();
}

// --------------------------------------------------------------------------------
// definitions for ::geometry
// --------------------------------------------------------------------------------

// -- points ----------------------------------------------------------------------

inline cv::Mat
geometry::draw_pt(const cv::Mat& img, cv::Point pt, cv::Scalar color)
{
	cv::Mat img_out;

	if (img.channels() == 1)
		cv::cvtColor(img, img_out, CV_GRAY2BGR);
	else
		img_out = img.clone();

	cv::circle(img_out, cv::Point(pt.x, pt.y), 3, color);

	return img_out;
}

inline cv::Mat
geometry::draw_pts(const cv::Mat& img, std::vector<cv::Point> pts, cv::Scalar color)
{
	cv::Mat img_out;

	if (img.channels() == 1)
		cv::cvtColor(img, img_out, CV_GRAY2BGR);
	else
		img_out = img.clone();

	for (const auto& pt : pts)
		cv::circle(img_out, cv::Point(pt.x, pt.y), 3, color);

	return img_out;
}

inline cv::Point
geometry::avg_pt(const std::vector<cv::Point>& pts)
{
	auto sum = std::accumulate(pts.begin(), pts.end(), cv::Point(0, 0));
	auto avg = cv::Point2f(sum.x / (float)pts.size(), sum.y / (float)pts.size());

	return static_cast<cv::Point>(avg);
}

inline std::optional<std::vector<cv::Point>>
geometry::pts_within_radius(std::vector<cv::Point>& pts, const cv::Point& center, float radius)
{
	if (radius < 0)
		throw std::runtime_error(ERR_NUM_NOT_POS);

	std::vector<cv::Point> pts_match;

	for (auto& pt : pts)
	{	
		auto diff_x = std::abs(pt.x - center.x);
		auto diff_y = std::abs(pt.y - center.y);

		if (std::pow(diff_x, 2) + std::pow(diff_y, 2) < std::pow(radius, 2))
			pts_match.push_back(pt);
	}

	if (pts_match.empty())
		return std::nullopt;
	
	else
		return pts_match;
}

inline bool
geometry::pts_connected(const cv::Mat& img, const cv::Point& a,  const cv::Point& b, float max_deviation)
{
	auto img_copy     = img.clone();
	auto num_non_zero = cv::countNonZero(img_copy);

	cv::line(img_copy, a, b, {255}, 1, 8);

	auto deviation = std::abs((cv::countNonZero(img_copy) - num_non_zero) / (float)num_non_zero);

	// std::cout << "before: " << num_non_zero << " | after: " << cv::countNonZero(img_copy) << std::endl;
	// std::cout << "deviation: " << deviation << " | max_deviation: " << max_deviation << " | connected: " << (deviation <= max_deviation) << std::endl;
	
	return (deviation <= max_deviation);

	// return (cv::countNonZero(img_copy) == num_non_zero);
}

inline std::vector<vertex_t>
geometry::extract_vertices(const cv::Mat& img, bool remove_duplicates, float duplicates_radius)
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
	
	// remove/average duplicates within radius
	if (remove_duplicates)
	{	
		std::vector<vertex_t> vec_vertices_avg;
		std::vector<vertex_t> vec_vertices_checked;
		for (auto& pt : vec_vertices)
		{
			// check whether point already belongs to a cluster
			if (std::any_of(vec_vertices_checked.begin(), vec_vertices_checked.end(), [&](const auto& a){
				return a == pt;
			}))	continue;

			// check whether point is on boundary
			if (pt.x == 0 || pt.y == 0)
				continue;

			// perform radius mathching
			if (auto vec_pts = pts_within_radius(vec_vertices, pt, duplicates_radius))
			{
				// average point
				auto pt_avg = avg_pt(vec_pts.value());
				vec_vertices_avg.push_back(pt_avg);

				// add matching points (cluster)
				vec_vertices_checked.insert(vec_vertices_checked.end(), vec_pts.value().begin(), vec_pts.value().end());
			}
		}

		vec_vertices = vec_vertices_avg;
	}


	// count number of vertices
	auto num_vertices = vec_vertices.size();

	return vec_vertices;
}

// -- line segment intersection ---------------------------------------------------

inline box_t
line_t::bounding_box() const
{
	box_t box;

	box[0] = cv::Point(std::min(this->from.x, this->to.x), std::min(this->from.y, this->to.y));
	box[1] = cv::Point(std::max(this->from.x, this->to.x), std::max(this->from.y, this->to.y));

	return box;
}

inline size_t
geometry::count_duplicate_lines(std::vector<line_t> vec_lines)
{
	std::sort(vec_lines.begin(), vec_lines.end(), [](const auto& a, const auto& b){
		return a.from.x < b.from.x;
	});
	
	std::sort(vec_lines.begin(), vec_lines.end(), [](const auto& a, const auto& b){
		return a.from.y < b.from.y;
	});

	auto unique_end = std::unique(vec_lines.begin(), vec_lines.end(), [](const auto& a, const auto& b){
		return (a.from == b.from && a.to == b.to);
	});

	return (unique_end == vec_lines.end() ? 0 : std::distance(vec_lines.begin(), unique_end));
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
	return segment_intersect(a.from, a.to, b.from, b.to);
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