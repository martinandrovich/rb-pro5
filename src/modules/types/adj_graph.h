#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "../../constants.h"

// --------------------------------------------------------------------------------
// declarations for ::adj_graph_t
// --------------------------------------------------------------------------------

class adj_graph_t
{

public:

	// constructs

	enum decomp_type
	{
		trapezoidal,
		boustraphedon,
		voronoi
	};

	struct edge_t
	{
		size_t start_x = 0;
		size_t start_y = 0;
		size_t end_x   = 0;
		size_t end_y   = 0;
		size_t length  = 0;
	};

	struct vertex_t
	{
		size_t x = 0;
		size_t y = 0;
	};

	// methods

	static adj_graph_t
	gen_adj_graph(const cv::Mat& img, decomp_type type = trapezoidal);

	cv::Mat
	get_img_lines() const;

	cv::Mat
	get_img_vertices() const;

private:

	// methods
	
	static std::vector<cv::Vec4i>
	extract_lines(const cv::Mat& img);

	static std::vector<edge_t>
	extract_edges(const cv::Mat& img);

	static std::vector<vertex_t>
	extract_vertices(const cv::Mat& img);

	static std::vector<vertex_t>
	extract_vertices(const std::vector<edge_t>& edges);


	// members

	cv::Mat img;

	std::vector<edge_t> edges;
	std::vector<vertex_t> vertices;

};

// --------------------------------------------------------------------------------
// method defintions for ::adj_graph_t
// --------------------------------------------------------------------------------

inline adj_graph_t
adj_graph_t::gen_adj_graph(const cv::Mat& img, decomp_type type)
{

	// create adjacency graph
	adj_graph_t adj_graph;

	// store image
	adj_graph.img = img;

	// extract vertices
	adj_graph.vertices = extract_vertices(img);

	std::cout << "number of vertices: " << adj_graph.vertices.size() << std::endl;
	show_img("vertices", adj_graph.get_img_vertices());

	// cell decomoposition
	if (type == trapezoidal)
		std::cout << "trapezoidal cell decomp" << std::endl;

	// return graph
	return std::move(adj_graph);
}

inline cv::Mat
adj_graph_t::get_img_vertices() const
{
	cv::Mat img_vertices;
	cv::cvtColor(img, img_vertices, CV_GRAY2BGR);

	for (const auto& v : this->vertices)
		cv::circle(img_vertices, cv::Point(v.x, v.y), 3, cv::Scalar(255, 0, 255));

	return std::move(img_vertices);
}

inline std::vector<cv::Vec4i>
adj_graph_t::extract_lines(const cv::Mat& img)
{

	// https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
	// https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html

	// must be grayscale
	if (img.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_canny, img_lines;

	// convert output image to color
	cv::cvtColor(img, img_lines, CV_GRAY2BGR);

	// detect edges using canny
	cv::Canny(img, img_canny, 50, 100, 3);

#if 0

	std::vector<cv::Vec2f> vec_lines;
	
	// perform standard hough line extraction (shit)
	cv::HoughLines(img_canny, vec_lines, 1, CV_PI/180, 75, 0, 0);

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

		cv::line(img_lines, pt1, pt2, cv::Scalar(255,0,255), 1, CV_AA);
	}

#else
	
	std::vector<cv::Vec4i> vec_lines;

	// perform probabilistic hough line extraction
	cv::HoughLinesP(
		img_canny,    // input img (from edge detector)
		vec_lines,    // ouput vector of [x_start, y_start, x_end, y_end] for each line
		0.01,            // resolution of parameter rho in pixels
		0.01,      // resolution of theta in radians
		1,           // minimum number of intersections to “detect” a line
		10,           // minimum number of points that can form a line
		10            // maximum gap between two points to be considered in the same line
	);

	// draw lines
	for (const auto& l : vec_lines)
		cv::line(img_lines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,255), 1, CV_AA);

#endif

	// show images
	cv::imshow("canny", img_canny);
	cv::imshow("lines", img_lines);
	cv::waitKey();	
	
	return cv::Mat();
}

inline std::vector<adj_graph_t::vertex_t>
adj_graph_t::extract_vertices(const cv::Mat& img)
{

	// must be grayscale
	if (img.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// vector of contours
	std::vector<std::vector<cv::Point>> vec_contours;
	cv::findContours(img, vec_contours, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1);

	// flatten vector of contours into vector of vertices
	std::vector<vertex_t> vec_vertices;
	for (const auto& vec : vec_contours)
		for (const auto& vertex : vec)
			vec_vertices.emplace_back(vertex_t{(size_t)vertex.x, (size_t)vertex.y});

	return std::move(vec_vertices);
}