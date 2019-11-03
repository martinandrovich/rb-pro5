#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../../constants.h"

// --------------------------------------------------------------------------------
// declarations for ::adj_graph_t
// --------------------------------------------------------------------------------

class adj_graph_t
{

public:

	enum decomp_type
	{
		trapezoidal,
		boustraphedon,
		voronoi
	};

	static adj_graph_t
	gen_adj_graph(const cv::Mat& img, decomp_type type = trapezoidal);

	cv::Mat
	get_img_lines();

private:

	static std::vector<cv::Vec4i>
	extract_lines(const cv::Mat& img);

	cv::Mat img;
	std::vector<cv::Vec4i> lines;

};

// --------------------------------------------------------------------------------
// method defintions for ::adj_graph_t
// --------------------------------------------------------------------------------

inline adj_graph_t
adj_graph_t::gen_adj_graph(const cv::Mat& img, decomp_type type)
{

	// create adjacency graph
	adj_graph_t adj_graph;

	// check image
	if (img.empty())
		throw std::runtime_error(ERR_IMG_EMPTY);

	// store image
	adj_graph.img = img;

	// extract lines
	adj_graph.lines = extract_lines(img);

	// cell decomoposition
	if (type == trapezoidal)
		std::cout << "trapezoidal cell decomp" << std::endl;

	// return graph
	return std::move(adj_graph);
}

inline std::vector<cv::Vec4i>
adj_graph_t::extract_lines(const cv::Mat& img)
{

	// https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html

	// must be grayscale
	if (img.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_canny, img_lines;
	std::vector<cv::Vec4i> vec_lines;

	// convert output image to color
	cv::cvtColor(img, img_lines, CV_GRAY2BGR);

	// detect edges using canny
	cv::Canny(img, img_canny, 50, 200, 3);

#if 0
	
	// perform standard hough line extraction (shit)
	std::vector<cv::Vec2f> vec_lines;
	cv::HoughLines(img_canny, vec_lines, 1, CV_PI/180, 100, 0, 0);

	// draw lines
	for (size_t i = 0; i < vec_lines.size(); i++)
	{
		float rho = vec_lines[i][0], theta = vec_lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;

		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));

		cv::line(img_lines, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
	}

#else
	
	// perform probabilistic hough line extraction
	cv::HoughLinesP(
		img_canny,    // input img (from edge detector)
		vec_lines,    // ouput vector of [x_start, y_start, x_end, y_end] for each line
		1,            // resolution of parameter rho in pixels
		CV_PI/180,    // resolution of theta in radians
		50,           // minimum number of intersections to “detect” a line
		50,           // minimum number of points that can form a line
		10            // maximum gap between two points to be considered in the same line
	);

	// draw lines
	for (const auto& l : vec_lines)
		cv::line(img_lines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);

#endif

	// show images
	cv::imshow("input", img);
	cv::imshow("detected lines", img_lines);
	cv::waitKey();	
	
	return cv::Mat();
}