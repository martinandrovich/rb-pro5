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

struct gvd_t
{
	cv::Mat img;
};

// --------------------------------------------------------------------------------
// declarations for ::geometry
// --------------------------------------------------------------------------------

namespace geometry
{

	// -- morph structuring elements  -------------------------------------------------
	
	// structuring elements
	inline auto str_elm_rect_2x2 = cv::getStructuringElement(cv::MORPH_RECT,    cv::Size(2, 2));
	inline auto str_elm_rect_5x5 = cv::getStructuringElement(cv::MORPH_RECT,    cv::Size(5, 5));
	inline auto str_elm_rect_9x9 = cv::getStructuringElement(cv::MORPH_RECT,    cv::Size(9, 9));
	inline auto str_elm_cros_2x2 = cv::getStructuringElement(cv::MORPH_CROSS,   cv::Size(2, 2));
	inline auto str_elm_elps_2x2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
	
	// -- brushfire & GVD -------------------------------------------------------------

	bool
	eight_adj_contains(const cv::Mat& img, const cv::Point& pt, size_t value);

	cv::Mat
	brushfire(const cv::Mat& img_map);

	cv::Mat
	gvd(const cv::Mat& img_map);

	cv::Mat
	gvd_bf(const cv::Mat& img_map, cv::Mat img_bf = cv::Mat());

	cv::Mat
	gvd_opencv(const cv::Mat& img_map);

	cv::Mat
	gvd_simple(const cv::Mat& img_map);

	gvd_t
	gvd_graph(const cv::Mat& img_gvd);

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
	test_gvd_draw_edges2();
	
	void
	test_brushfire_and_gvd();

	void
	test_segment_intersect();
}

// --------------------------------------------------------------------------------
// definitions for ::geometry
// --------------------------------------------------------------------------------

// -- brushfire & GVD -------------------------------------------------------------

inline bool
geometry::eight_adj_contains(const cv::Mat& img, const cv::Point& pt, size_t value)
{
	// absolute bounds and starting position for seeking pixel
	// if seeking pixel starts in a corner or near image edge
	
	size_t bound_row, bound_col, start_col, start_row, num_adj;

	bound_row = (pt.y >= (img.rows - 1)) ? img.rows : pt.y + 2;
	bound_col = (pt.x >= (img.cols - 1)) ? img.cols : pt.x + 2;
	start_col = pt.x - ((pt.x <= 0) ? 0 : 1);
	start_row = pt.y - ((pt.y <= 0) ? 0 : 1);

	// seek the kernel grid; return true if val is found
	for (size_t row = start_row; row < (size_t)bound_row; row++)
	{
		for (size_t col = start_col; col < (size_t)bound_col; col++)
		{
			if (img.at<uchar>(cv::Point(col, row)) == (uchar)value)
				return true;
		}
	}

	return false;
}

inline cv::Mat
geometry::brushfire(const cv::Mat& img_map)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img, img_map_inv;

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

	std::vector<cv::Point> gvd_pts;

	// brushfire algorithm
	// iterate all 0-valued pixels and increment n each (complete) iteration
	// if zero-th pixel has the value n within its adjacency, then set that pixel to n + 1
	for (size_t n = 1; num_zero_px != 0 ; ++n)
	{
		if (n > 255)
			throw std::runtime_error("Value of n has exceeded 255.");

		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (pixel == 0 && eight_adj_contains(img, pos, n))
			{
				pixel = n + 1;
				--num_zero_px;
			}
		});
	}

	return img;
}

inline cv::Mat
geometry::gvd(const cv::Mat& img_map)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img, img_map_inv, img_labels, img_gvd, img_gvd_dl;
	
	// invert image; make obstacles white
	cv::bitwise_not(img_map, img_map_inv);

	// binarize the inverse
	img = cv::min(img_map_inv, 1);

	// count zero-value pixels
	auto num_zero_px = img.rows * img.cols - cv::countNonZero(img);

	// segregate map into obstacles (map of obstacles)
	auto n_labels = cv::connectedComponents(img_map_inv, img_labels, 8);

	std::map<int, std::vector<cv::Point>> obstacles;

	for(int r = 0; r < img_map.rows; ++r)
	{
		for(int c = 0; c < img_map.cols; ++c)
		{
			auto label = img_labels.at<int>(r, c);
			obstacles[label].emplace_back(cv::Point(c, r));
		}
	}

	// -------------------------------------------------------------

	struct point_t
	{
		std::map<int, int> asgmts;

		cv::Point pos;
		char      value    = 0;
		bool      gvd      = false;
	};

	auto is_expandable = [&](const cv::Point& pt)
	{
		uint num_expand_px = 0;

		iterate_8adj(img, pt, [&](auto& pos, auto& pixel) {
			num_expand_px += (uint)(pixel == 0);
		});

		return (bool)num_expand_px;
	};

	auto num_4adj = [&](const cv::Point& pt, const size_t n)
	{
		size_t count = 0;

		iterate_4adj(img, pt, [&](auto& pos, auto& pixel) {
			count += (size_t)(pixel == n && pos != pt);
		});

		return count;
	};

	// -------------------------------------------------------------

	img_gvd = cv::Mat::zeros(img_map.size(), CV_8U);
	std::vector<std::vector<point_t>> map(img.cols, std::vector<point_t>(img.rows));
	size_t n = 0;

	while (num_zero_px > 0)
	{
		// increment n (always starts at n = 1)
		n++;

		// expand all obstacles
		for (size_t label = 1; label < n_labels; ++label)
		{
			auto starting_size = obstacles[label].size();

			for (size_t i = 0; i < starting_size; ++i)
			{
				// current point and pixel value
				auto& pt = obstacles[label][i];
				auto& px = img.at<uchar>(pt);

				// is expandable?
				if (px < n)
					continue;

				// expand
				iterate_8adj(img, pt, [&](auto& pos, auto& pixel)
				{
					if (pixel == 0 || pixel == n + 1)
					{
						auto& p = map[pos.x][pos.y];

						p.value = n + 1;
						p.asgmts[label]++;
						p.pos = pos;

						if (pixel == 0)
						{
							pixel = n + 1;
							num_zero_px--;
							obstacles[label].push_back(pos);
						}
					}
				});
			}
		}

		// check collisions
		for (auto& outer : map)
		{
			for (auto& pt : outer)
			{
				if (pt.value != 0)
				{
					// collision with other
					if (pt.asgmts.size() > 1)
						pt.gvd = true;

					// collision with self
					if (std::any_of(pt.asgmts.begin(), pt.asgmts.end(), [](std::pair<int, int> e) {
						return e.second > 4;
					})) pt.gvd = true;

					// touches self (number of self adj pixels > 2)
					if (num_4adj(pt.pos, n + 1) > 2)
						pt.gvd = true;
				}
			}
		}

		// apply GVD's
		for (auto& outer : map)
			for (auto& pt : outer)
				if (pt.gvd)
				{
					img.at<uchar>(pt.pos) = 255;
					img_gvd.at<uchar>(pt.pos) = 255;
				}
	}

	// show_img("gvd + bf (custom)", img);

	// -------------------------------------------------------------

	// purge double lines

	// first identify any double lines by eroding thick parts of the GVD
	cv::erode(img_gvd, img_gvd_dl, str_elm_elps_2x2);

	// remove the detected lines from the GVD
	// invert the found double lines and BITWISE_AND it with the original GVD
	cv::bitwise_not(img_gvd_dl, img_gvd_dl);
	cv::bitwise_and(img_gvd, img_gvd_dl, img_gvd);
	// show_img("gvd: custom (thin)", img_gvd);

	// make GVD lines thicker, i.e. dilute the black pixels (erode on inverse image)
	cv::dilate(img_gvd, img_gvd, str_elm_rect_2x2);
	// show_img("gvd: custom (thickened)", img_gvd);

	// close any minor gaps
	cv::morphologyEx(img_gvd, img_gvd, cv::MORPH_CLOSE, str_elm_rect_2x2);
	// show_img("gvd: custom (gaps closed)", img_gvd);

	// remove any noise
	cv::morphologyEx(img_gvd, img_gvd, cv::MORPH_OPEN, str_elm_elps_2x2);
	// show_img("gvd: custom (noise removal)", img_gvd);

	// invert GVD, making the GVD edge black
	cv::bitwise_not(img_gvd, img_gvd);

	// return gvd image
	return img_gvd;
}

inline void
gradient_test()
{
	cv::Mat img, img_labels, img_laplace, img_sobel, img_canny, img_gvd;

	img = load_img(PATH_IMG_GRAD_TEST, cv::IMREAD_GRAYSCALE);

	std::cout << img << std::endl;
	show_img("image", img);

	// detect edges using canny
	cv::Canny(img, img_canny, 50, 100, 3);
	std::cout << img_canny << std::endl;
	show_img("gvd: canny", img_canny);
}

inline cv::Mat
geometry::gvd_bf(const cv::Mat& img_map, cv::Mat img_bf)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_map_inv, img_laplace, img_sobel, img_gvd, img_canny;

	// auto self_collision = [&](const cv::Point& pt, const size_t n)
	// {
	// 	// boundary check for 3x3 kernel
	// 	if ((pt.x - 1) < 0 || (pt.y - 1) < 0 || (pt.x + 1 > img.cols) || (pt.y + 1 > img.rows))
	// 		return false;

	// 	const auto& tl = img.at<uchar>({pt.x - 1, pt.y - 1 });
	// 	const auto& tm = img.at<uchar>({pt.x    , pt.y - 1 });
	// 	const auto& tr = img.at<uchar>({pt.x + 1, pt.y - 1 });
	// 	const auto& ml = img.at<uchar>({pt.x + 1, pt.y     });
	// 	const auto& mm = img.at<uchar>({pt.x    , pt.y     });
	// 	const auto& mr = img.at<uchar>({pt.x - 1, pt.y     });
	// 	const auto& bl = img.at<uchar>({pt.x - 1, pt.y + 1 });
	// 	const auto& bm = img.at<uchar>({pt.x    , pt.y + 1 });
	// 	const auto& br = img.at<uchar>({pt.x + 1, pt.y + 1 });

	// 	// match pattern within kernel
	// 	return 
	// 	(
	// 		(mm == n + 1 && mm == tm && bm == (mm - 1))
	// 	);		
	// };

	// compute inverse
	cv::bitwise_not(img_map, img_map_inv);

	// compute brushfire if not specified
	if (img_bf.empty())
		img_bf = brushfire(img_map);

	// use Canny to find gradient, i.e. the GVD
	// cv::Canny(img_bf, img_canny, 0, 0, 5, true);
	// show_img("gvd: canny", img_canny);

	// use Laplace operator to find gradient, i.e. the GVD
	cv::Laplacian(img_bf, img_laplace, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(img_laplace, img_laplace);

	// fix laplace
	// remove obstacle outlinees from GVD (subtract the inverse), invert the GVD, making the edges black
	cv::threshold(img_laplace, img_laplace, 0, 255, cv::THRESH_BINARY);
	img_laplace -= img_map_inv;
	cv::bitwise_not(img_laplace, img_laplace);
	
	img_gvd = img_laplace;

	// show_img("gvd: laplace", img_laplace);

	// // use Sobel operator to find gradient
	// cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y;

	// // gradient X
	// cv::Sobel(img_bf, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	// cv::convertScaleAbs(grad_x, abs_grad_x);

	// // gradient Y
	// cv::Sobel(img_bf, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
	// cv::convertScaleAbs(grad_y, abs_grad_y);

	// // total gradient (approximate)
	// cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, img_sobel);

	// // fix sobel
	// cv::threshold(img_sobel, img_sobel, 0, 255, cv::THRESH_BINARY);
	// img_sobel += img_map_inv;
	// show_img("gvd: sobel", img_sobel);
	
	// // combine sobel and laplace
	// cv::bitwise_and(img_laplace, img_sobel, img_gvd);
	
	return img_gvd;
}

inline cv::Mat
geometry::gvd_opencv(const cv::Mat& img_map)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_map_inv, img_obs, img_gvd, img_laplace, img_laplace2;

	// extract obstacles (black pixels)
	std::vector<cv::Point> vec_obs;
	cv::bitwise_not(img_map, img_map_inv);
	cv::findNonZero(img_map_inv, vec_obs);

	// vec_obs = extract_vertices(img_map);

	// rectangle to be used with Subdiv2D
	cv::Rect rect(0, 0, img_map.cols, img_map.rows);

	// instance of Subdiv2D
	cv::Subdiv2D subdiv(rect);

	// insert points into subdiv
	for (const auto& pt : vec_obs)
		subdiv.insert(pt);

	// draw/create gvd

	img_gvd = cv::Mat::zeros(img_map.rows, img_map.cols, CV_8UC3);

	std::vector<std::vector<cv::Point2f>> facets;
	std::vector<cv::Point2f> centers;

	subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);
 
	std::vector<cv::Point> ifacet;
	std::vector<std::vector<cv::Point>> ifacets(1);
 
	for (size_t i = 0; i < facets.size(); i++)
	{
		ifacet.resize(facets[i].size());
		
		for( size_t j = 0; j < facets[i].size(); j++ )
			ifacet[j] = facets[i][j];
 
		// cv::Scalar color(127, 127, 127);
		// color[0] = rand() & 255;
		// color[1] = rand() & 255;
		// color[2] = rand() & 255;

		//fillConvexPoly(img_gvd, ifacet, color, 8, 0);
 
		ifacets[0] = ifacet;
		cv::polylines(img_gvd, ifacets, true, cv::Scalar(255), 1, 4);
		//cv::circle(img_gvd, centers[i], 3, cv::Scalar(), CV_FILLED, CV_AA, 0);
	}

	cv::Mat channels[3];
	cv::split(img_gvd, channels);
	img_gvd = channels[0];

	show_img("gvd: opencv (original)", img_gvd);

	cv::Laplacian(img_gvd, img_laplace, CV_16S, 5, 1, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(img_laplace, img_laplace);
	cv::threshold(img_laplace, img_laplace, 0, 255, cv::THRESH_BINARY);

	// morph dilation
	cv::dilate(img_laplace, img_laplace, str_elm_elps_2x2);
	cv::bitwise_not(img_laplace, img_laplace);
	cv::dilate(img_laplace, img_laplace, str_elm_elps_2x2);

	// expand map walls and mask the GVD
	cv::dilate(img_map_inv, img_map_inv, str_elm_rect_9x9);
	cv::bitwise_or(img_laplace, img_map_inv, img_laplace);

	// add map
	// cv::bitwise_and(img_map, img_laplace, img_laplace);

	return img_laplace;
	return img_gvd;
}

inline cv::Mat
geometry::gvd_simple(const cv::Mat& img_map)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	cv::Mat img     = img_map.clone();
	cv::Mat img_gvd = cv::Mat::zeros(img_map.rows, img_map.cols, CV_8U);

	// iterate all pixels of image
	iterate_mat(img, [&](auto& pos, auto& pixel)
	{
		auto shortest_dist = INFINITY;
		auto obs_count     = 0;

		// for all white pixels
		if (pixel == 255) iterate_mat(img, [&](auto& pos2, auto& pixel2)
		{

			if (pixel2 != 0)
				return;

			auto dx   = std::abs(pos.x - pos2.x);
			auto dy   = std::abs(pos.y - pos2.y);
			auto dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

			if (dist == shortest_dist)
				obs_count++;
		
			else
			if (dist < shortest_dist)
			{
				shortest_dist = dist;
				obs_count = 1;
			}

		});

		// std::cout << pos << " | " << shortest_dist << " | " << obs_count << std::endl;

		// set pixel value
		img_gvd.at<uchar>(pos) = (obs_count > 1) ? 127 : (uchar)shortest_dist;
	});

	//std::cout << img_gvd << std::endl;
	return img_gvd;
}

inline gvd_t
geometry::gvd_graph(const cv::Mat& img_gvd)
{

}

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
geometry::test_gvd_draw_edges()
{
	// variables
	cv::Mat img, img_gvd, img_gvd_inv, img_out;

	// big world
	img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	scale_img(img, DIM_BIGWORLD, SCALE_METER_PER_PX);

	// gvd using custom implementation
	img_gvd = gvd(img);
	show_img("gvd (custom)", img_gvd);

	// vertices
	auto vec_vert = extract_vertices(img_gvd, true, GVD_VERTEX_RADIUS);
	auto img_vert = draw_pts(img_gvd, vec_vert);
	show_img("vertices (custom)", img_vert);

	// inverse GVD and dilate
	cv::bitwise_not(img_gvd, img_gvd_inv);
	cv::dilate(img_gvd_inv, img_gvd_inv, str_elm_rect_2x2);
	show_img("gvd (custom, inv, dilated)", img_gvd_inv);

	// create output
	cv::cvtColor(img_gvd, img_out, cv::COLOR_GRAY2BGR);
	img_out.setTo(cv::Scalar(255, 255, 255));

	// iterate all vertices
	for (const auto& v1 : vec_vert)
	{
		// for each vertex, try drawing to that
		for (const auto& v2 : vec_vert)
		{
			if (v1 == v2)
				continue;

			if (pts_connected(img_gvd_inv, v1, v2, 0))
				cv::line(img_out, v1, v2, {0, 0, 255}, 1, 8);
		}
	}

	img_out = draw_pts(img_out, vec_vert);
	show_img("gvd w/ edges", img_out);
}

inline void
geometry::test_gvd_draw_edges2()
{
	// variables
	cv::Mat img, img_gvd, img_edges, img_canny, img_out;

	// big world
	img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	scale_img(img, DIM_BIGWORLD, SCALE_METER_PER_PX);

	// gvd using custom implementation
	img_gvd = gvd(img);
	show_img("gvd (custom)", img_gvd);

	// vertices
	auto vec_vert = extract_vertices(img_gvd, true, GVD_VERTEX_RADIUS);
	auto img_vert = draw_pts(img_gvd, vec_vert);
	show_img("vertices (custom)", img_vert);

	// create empty output
	cv::cvtColor(img_gvd, img_out, cv::COLOR_GRAY2BGR);
	img_out.setTo(cv::Scalar(255, 255, 255));

	// canny
	// cv::Canny(img_gvd, img_canny, 50, 100, 3);
	// show_img("gvd: canny", img_canny);

	// manual edges
	img_edges = img_gvd.clone();
	cv::bitwise_not(img_edges, img_edges);
	cv::erode(img_edges, img_edges, str_elm_rect_2x2);
	show_img("gvd: edges", img_edges);

#if 1

	// perform standard hough line extraction (shit)
	std::vector<cv::Vec2f> vec_lines;
	cv::HoughLines(img_edges, vec_lines, 1, CV_PI/180, 75, 0, 0);

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

		cv::line(img_out, pt1, pt2, cv::Scalar(255, 0, 255), 1, 8);
	}

#else

	// perform probabilistic hough line extraction
	std::vector<cv::Vec4i> vec_lines;
	cv::HoughLinesP(
		img_edges,    // input img (from edge detector)
		vec_lines,    // ouput vector of [x_start, y_start, x_end, y_end] for each line
		1,            // resolution of parameter rho in pixels
		CV_PI/180,    // resolution of theta in radians
		10,           // minimum number of intersections to “detect” a line
		5,            // minimum number of points that can form a line
		0             // maximum gap between two points to be considered in the same line
	);

	// draw lines
	for (const auto& l : vec_lines)
		cv::line(img_out, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 255), 1, 8);

#endif

	// img_out = draw_pts(img_out, vec_vert);
	show_img("gvd w/ edges", img_out);

}

inline void
geometry::test_brushfire_and_gvd()
{
	// load and scale image

	// small world
	//auto img = load_img(PATH_IMG_SMALLWORLD, cv::IMREAD_GRAYSCALE);
	//scale_img(img, DIM_SMALLWORLD, SCALE_METER_PER_PX);

	// big world
	auto img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	scale_img(img, DIM_BIGWORLD, SCALE_METER_PER_PX);

	// manual
	// auto img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	// cv::resize(img, img, cv::Size(), 5.f, 5.f, cv::INTER_NEAREST);
	
	// gradient test
	// gradient_test();

	// show input
	show_img("map", img);

	// vertex edges test
	test_gvd_draw_edges2();

	// brushfire
	auto img_bf = brushfire(img);
	// show_img("brushfire", img_bf);

	// gvd using brushfire
	auto img_gvdbf = gvd_bf(img, img_bf);
	// show_img("gvd (bf)", img_gvdbf);

	// gvd using custom implementation
	auto img_gvd = gvd(img);
	show_img("gvd (custom)", img_gvd);

	// opencv GVD
	auto img_gvdcv = gvd_opencv(img);
	show_img("gvd (opencv)", img_gvdcv);

	// vertices (bf)
	auto vec_vert_bf = extract_vertices(img_gvdbf, true, GVD_VERTEX_RADIUS);
	auto img_vert_bf = draw_pts(img_gvdbf, vec_vert_bf);
	show_img("vertices (bf) ", img_vert_bf);

	// vertices (custom)
	auto vec_vert = extract_vertices(img_gvd, true, GVD_VERTEX_RADIUS);
	auto img_vert = draw_pts(img_gvd, vec_vert);
	show_img("vertices (custom)", img_vert);
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