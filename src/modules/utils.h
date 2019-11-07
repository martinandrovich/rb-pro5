#pragma once

#include <string>
#include <vector>
#include <stdexcept>

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

// segment intersection

inline constexpr auto EPSILON = 0.000001;

struct line_t
{
	cv::Point from;
	cv::Point to;

	std::array<cv::Point, 2>
	bounding_box()
	{
		std::array<cv::Point, 2> box;

		box[0] = cv::Point(std::min(from.x, to.x), std::min(from.y, to.y));
		box[1] = cv::Point(std::max(from.x, to.x), std::max(from.y, to.y));

		return box;
	}
};

inline double
cross(cv::Point a, cv::Point b)
{
	return a.x * b.y - b.x * a.y;
}

inline bool
doBoundingBoxesIntersect(std::array<cv::Point, 2> a, std::array<cv::Point, 2> b)
{
	return a[0].x <= b[1].x && a[1].x >= b[0].x && a[0].y <= b[1].y	&& a[1].y >= b[0].y;
}

inline bool
isPointOnLine(line_t& line, cv::Point pt)
{
	// move the image, so that a.first is on (0|0)
	auto line_temp = line_t{ { 0, 0 }, cv::Point(line.to.x - line.from.x, line.to.y - line.from.y) };	
	auto pt_temp   = cv::Point(pt.x - line.from.x, pt.y - line.from.y);

	return (std::abs(cross(line_temp.to, pt_temp)) < EPSILON);
}

inline bool
isPointRightOfLine(line_t line, cv::Point pt)
{
	// move the image, so that a.first is on (0|0)
	auto line_temp = line_t{ { 0, 0 }, cv::Point(line.to.x - line.from.x, line.to.y - line.from.y) };	
	auto pt_temp   = cv::Point(pt.x - line.from.x, pt.y - line.from.y);

	return (cross(line_temp.to, pt_temp) < 0);
}

inline bool
lineSegmentTouchesOrCrossesLine(line_t& a, line_t& b)
{
	return isPointOnLine(a, b.from) || isPointOnLine(a, b.to) || (isPointRightOfLine(a, b.from) ^ isPointRightOfLine(a, b.to));
}

inline bool
segment_intersect(line_t& a, line_t& b)
{
	auto box1 = a.bounding_box();
	auto box2 = b.bounding_box();

	return doBoundingBoxesIntersect(box1, box2)	&& lineSegmentTouchesOrCrossesLine(a, b) && lineSegmentTouchesOrCrossesLine(b, a);
}