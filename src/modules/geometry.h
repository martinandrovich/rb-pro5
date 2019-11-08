#pragma once

#include <cmath>
#include <array>
#include <stdexcept>
#include <optional>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../constants.h"
#include "utils.h"

// --------------------------------------------------------------------------------
// declarations for ::geometry
// --------------------------------------------------------------------------------

namespace geometry
{
	
	// https://martin-thoma.com/how-to-check-if-two-line-segments-intersect/

	inline constexpr auto EPSILON = 1e-6;

	using box_t = std::array<cv::Point, 2>;

	struct line_t
	{
		cv::Point from;
		cv::Point to;

		box_t
		bounding_box() const;
	};

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

	std::optional<cv::Point>
	segment_intersect_at(const line_t& a, const line_t& b);

	bool
	ccw(const cv::Point& A, const cv::Point& B, const cv::Point& C);

	bool
	segment_intersect_v2(const cv::Point& A, const cv::Point& B, const cv::Point& C, const cv::Point& D);

	void
	test_segment_intersect();
}

// --------------------------------------------------------------------------------
// definitions for ::geometry
// --------------------------------------------------------------------------------

inline geometry::box_t
geometry::line_t::bounding_box() const
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
geometry::segment_intersect(const line_t& a, const line_t& b)
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

	if (not segment_intersect(a, b))
		return std::nullopt;

	else
		throw std::runtime_error(ERR_NO_IMPL);
		// return cv::Point(0,0);
}

inline bool
geometry::ccw(const cv::Point& A, const cv::Point& B, const cv::Point& C)
{
	return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x);
}

inline bool
geometry::segment_intersect_v2(const cv::Point& A, const cv::Point& B, const cv::Point& C, const cv::Point& D)
{
	return ((ccw(A,C,D) != ccw(B,C,D)) && (ccw(A,B,C) != ccw(A,B,D)));
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
		intersect = segment_intersect(A, B);
	});

	benchmark<std::chrono::nanoseconds>([&]
	{
		intersect_v2 = segment_intersect_v2(A.from, A.to, B.from, B.to);
	});

	std::cout << "line intersect: "      << std::boolalpha << intersect << std::endl;
	std::cout << "line intersect (v2): " << std::boolalpha << intersect_v2 << std::endl;
}