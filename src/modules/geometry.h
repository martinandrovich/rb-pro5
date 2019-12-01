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

	void
	gvd_graph(const cv::Mat& img_gvd);

	void
	gvd_graph2(const cv::Mat& img_gvd);

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
	// http://www.tapas-project.eu/files/lau13ras.pdf
	// https://www.hindawi.com/journals/mpe/2014/456739/#B23

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
	cv::erode(img_gvd, img_gvd_dl, MORPH::STR_ELM_ELPS_2x2);

	// remove the detected lines from the GVD
	// invert the found double lines and BITWISE_AND it with the original GVD
	cv::bitwise_not(img_gvd_dl, img_gvd_dl);
	cv::bitwise_and(img_gvd, img_gvd_dl, img_gvd);
	// show_img("gvd: custom (thin)", img_gvd);

	// make GVD lines thicker, i.e. dilute the black pixels (erode on inverse image)
	cv::dilate(img_gvd, img_gvd, MORPH::STR_ELM_RECT_2x2);
	// show_img("gvd: custom (thickened)", img_gvd);

	// close any minor gaps
	cv::morphologyEx(img_gvd, img_gvd, cv::MORPH_CLOSE, MORPH::STR_ELM_RECT_2x2);
	// show_img("gvd: custom (gaps closed)", img_gvd);

	// remove any noise
	cv::morphologyEx(img_gvd, img_gvd, cv::MORPH_OPEN, MORPH::STR_ELM_ELPS_2x2);
	// show_img("gvd: custom (noise removal)", img_gvd);

	// invert GVD, making the GVD edge black
	cv::bitwise_not(img_gvd, img_gvd);

	// return gvd image
	return img_gvd;
}

inline cv::Mat
geometry::gvd_bf(const cv::Mat& img_map, cv::Mat img_bf)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_bf_norm, img_map_inv, img_laplace, img_sobel, img_sobel_abs, img_gvd;

	// compute inverse
	cv::bitwise_not(img_map, img_map_inv);

	// compute brushfire if not specified
	if (img_bf.empty())
		img_bf = brushfire(img_map);

	// normalize brushfire
	img_bf.convertTo(img_bf_norm, CV_32F);
	cv::normalize(img_bf_norm, img_bf_norm, 0, 1, cv::NORM_MINMAX);

	// use Sobel operator to find gradient
	cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y;

	// gradient X
	cv::Sobel(img_bf_norm, grad_x, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	// cv::Scharr(img_bf, grad_x, CV_32F, 1, 0, 1, 0, cv::BORDER_DEFAULT);

	// gradient Y
	cv::Sobel(img_bf_norm, grad_y, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
	// cv::Scharr(img_bf, grad_y, CV_32F, 0, 1, 1, 0, cv::BORDER_DEFAULT);

	// euclidean gradient
	cv::Mat grad_x2, grad_y2, grad, grad_norm;
	cv::pow(grad_x, 2, grad_x2);
	cv::pow(grad_y, 2, grad_y2);
	cv::sqrt(grad_x2 + grad_y2, grad);

	// normalize gradient
	cv::normalize(grad, grad_norm, 0, 1, cv::NORM_MINMAX);
	// show_img("gvd: sobel (normalized)", grad_norm);

	// threshold gradient
	cv::threshold(grad_norm, grad_norm, 0.8, 1.0, cv::THRESH_BINARY);
	// show_img("gvd: sobel (th)", grad_norm);

	// convert to 8-bit
	cv::convertScaleAbs(grad_norm, img_sobel);
	cv::threshold(img_sobel, img_sobel, 0, 255, cv::THRESH_BINARY);
	// show_img("gvd: sobel (8-bit)", img_sobel);

	// remove obstacles from gradient
	img_sobel += img_map_inv;
	// show_img("gvd: sobel (no obstacles)", img_sobel);
	
	// output
	img_gvd = img_sobel;
	// cv::imwrite("img_gvd.png", img_gvd);
	
	return img_gvd;
}

inline cv::Mat
geometry::gvd_opencv(const cv::Mat& img_map)
{

	// https://www.learnopencv.com/delaunay-triangulation-and-voronoi-diagram-using-opencv-c-python/

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
	cv::dilate(img_laplace, img_laplace, MORPH::STR_ELM_ELPS_2x2);
	cv::bitwise_not(img_laplace, img_laplace);
	cv::dilate(img_laplace, img_laplace, MORPH::STR_ELM_ELPS_2x2);

	// expand map walls and mask the GVD
	cv::dilate(img_map_inv, img_map_inv, MORPH::STR_ELM_RECT_9x9);
	cv::bitwise_or(img_laplace, img_map_inv, img_laplace);

	// add map
	// cv::bitwise_and(img_map, img_laplace, img_laplace);

	return img_laplace;
	return img_gvd;
}

inline void
geometry::gvd_graph(const cv::Mat& img_gvd)
{
	// assert grayscale image
	if (img_gvd.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img, img_out;
	std::vector<line_t> vec_edges;
	std::vector<vertex_t> vec_vertices;

	// copy gvd
	img = img_gvd.clone();

	// create empty output
	cv::cvtColor(img, img_out, cv::COLOR_GRAY2BGR);
	// img_out.setTo(cv::Scalar(255, 255, 255));

	// -------------------------------------------------------------

	// lambdas

	auto is_diag = [&](const cv::Point& pt) -> std::optional<cv::Point>
	{
		// boundary check
		if (not pt_within_boundary(img, pt))
			return std::nullopt;

		// pixel object
		auto pxl = PIXEL::pixel_t(img, pt);

		// perform diagonal check within 3x3 kernel

		if (not (pxl.mm() == PIXEL::BLACK && pxl.tm() == PIXEL::WHITE && pxl.ml() == PIXEL::WHITE && pxl.mr() == PIXEL::WHITE && pxl.bm() == PIXEL::WHITE))
			return std::nullopt;

		if (pxl.tl() == pxl.mm() && pxl.tr() == PIXEL::WHITE && pxl.bl() == PIXEL::WHITE && pxl.br() == PIXEL::WHITE)
			return PIXEL::DIR_NW;
			
		if (pxl.tr() == pxl.mm() && pxl.tl() == PIXEL::WHITE && pxl.bl() == PIXEL::WHITE && pxl.br() == PIXEL::WHITE)
			return PIXEL::DIR_NE;

		if (pxl.bl() == pxl.mm() && pxl.tr() == PIXEL::WHITE && pxl.tl() == PIXEL::WHITE && pxl.br() == PIXEL::WHITE)
			return PIXEL::DIR_SW;

		if (pxl.br() == pxl.mm() && pxl.tl() == PIXEL::WHITE && pxl.bl() == PIXEL::WHITE && pxl.tr() == PIXEL::WHITE)
			return PIXEL::DIR_SE;
	};

	auto count_black_px = [&](const cv::Point& pt)
	{
		size_t count = 0;

		iterate_3x3<uchar>(img, pt, [&](auto& pos, auto& pixel)
		{
			if (pixel == PIXEL::BLACK)
				count++;
		});

		return count;
	};

	auto find_diag = [&]()
	{

		// finding diagonals done on clone
		// such that new diagonals dont appear dynamically
		auto img_temp = img.clone();

		// iterate the original img, remove pixels from copy
		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (auto dir = is_diag(pos))
			{
				auto pos_start    = pos;
				auto pos_end      = cv::Point();
				auto pos_cur      = pos + dir.value();

				// add vertex to vector and make that pixel white
				vec_vertices.push_back(pos_start);
				img_temp.at<uchar>(pos_start) = PIXEL::WHITE;

				while (pt_within_boundary(img_temp, pos_cur) && count_black_px(pos_cur) <= 6 && img_temp.at<uchar>(pos_cur) == PIXEL::BLACK)
				{
					img_temp.at<uchar>(pos_cur) = PIXEL::WHITE;
					pos_cur += dir.value();
				}

				// calculate end point, add to vector of edges and make it black
				pos_end = pos_cur - dir.value();
				vec_edges.push_back({ pos_start, pos_end });
				// img_temp.at<uchar>(pos_end) = PIXEL::BLACK;
				img_temp.at<uchar>(pos_end) = PIXEL::WHITE;
				
				cv::line(img_out, pos_start, pos_end, { 0, 0, 255 }, 1, 8);
				img_out.at<cv::Vec3b>(pos_start) = { 255, 0, 255 };
				img_out.at<cv::Vec3b>(pos_end)   = { 255, 0, 255 };
			}
		});

		// when done, write back removed pixels to original img
		img = img_temp;
	};

	auto purge_doublelines = [&]()
	{
		cv::Mat img_dl;

		cv::dilate(img, img_dl, MORPH::STR_ELM_ELPS_2x2);
		cv::bitwise_not(img_dl, img_dl);
		cv::bitwise_or(img, img_dl, img);
	};

	auto fill_diag_gaps = [&]()
	{
		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (auto dir = is_diag(pos))
				img.at<uchar>(pos - dir.value()) = PIXEL::BLACK;
		});
	};

	auto find_horz = [&]()
	{

		bool on_line = false;

		cv::Point pos_start, pos_end;
		size_t dist = 0;

		iterate_rows(img, [&](auto& pos, auto& pixel)
		{

			// currently following a line
			if (on_line && pixel == PIXEL::BLACK)
			{
				dist++;
				
				if (dist > 1)
					pixel = PIXEL::WHITE;
			}
			// was following line but reached white pixel, stop following line
			else
			if (on_line && pixel == PIXEL::WHITE)
			{
				pos_end = pos;
				on_line = false;

				if (dist > 1)
					vec_edges.push_back({pos_start, pos_end});

				cv::line(img_out, pos_start, pos_end, { 0, 0, 255 }, 1, 8);
			}
			// encountered black pixel, start following line
			else
			if (pixel == PIXEL::BLACK)
			{
				pos_start = pos;
				dist = 0;
				on_line = true;
			}
		});
	};

	auto find_vert = [&]()
	{

		bool on_line = false;

		cv::Point pos_start, pos_end;
		size_t dist = 0;

		iterate_cols(img, [&](auto& pos, auto& pixel)
		{

			// currently following a line
			if (on_line && pixel == PIXEL::BLACK)
			{
				dist++;
				
				if (dist > 1)
					pixel = PIXEL::WHITE;
			}
			// was following line but reached white pixel, stop following line
			else
			if (on_line && pixel == PIXEL::WHITE)
			{
				pos_end = pos;
				on_line = false;

				if (dist > 1)
					vec_edges.push_back({pos_start, pos_end});

				cv::line(img_out, pos_start, pos_end, { 0, 0, 255 }, 1, 8);
			}
			// encountered black pixel, start following line
			else
			if (pixel == PIXEL::BLACK)
			{
				pos_start = pos;
				dist = 0;
				on_line = true;
			}
		});
	};

	// -------------------------------------------------------------

	// start
	cv::imwrite("assets/output/img_gvd_step0.png", img_gvd);

	// thin edges test
	auto img_gvd_thin = thin_edges(img_gvd);
	show_img("gvd: thinned", img_gvd_thin);
	cv::imwrite("assets/output/img_gvd_thin.png", img_gvd_thin);

	// img = img_gvd_thin;

	// find (most) diagonal edges
	find_diag();
	show_img("gvd: find diag (step 1)", img);
	cv::imwrite("assets/output/img_gvd_step1.png", img);

	// purge double edges
	purge_doublelines();
	cv::imwrite("assets/output/img_gvd_step2.png", img);
	show_img("gvd: remove doubles (step 2)", img);

	// fill diagonal gaps
	fill_diag_gaps();
	cv::imwrite("assets/output/img_gvd_step3.png", img);
	show_img("gvd: fill diag gaps (step 3)", img);

	// find horizontal edges
	find_horz();
	cv::imwrite("assets/output/img_gvd_step4.png", img);
	show_img("gvd: find horz (step 4)", img);

	// find vertical edges
	find_vert();
	cv::imwrite("assets/output/img_gvd_step5.png", img);
	show_img("gvd: find horz (step 5)", img);

	// find remaining diagonals
	;

	// re-construct gvd
	cv::Mat img_final = cv::Mat(img_gvd.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	
	for (const auto& e : vec_edges)
		cv::line(img_final, e.from, e.to, { 255, 0, 0 }, 1, 8);

	cv::imwrite("assets/output/img_gvd_reconstructed.png", img_final);
	show_img("gvd: reconstructed", img_final);	


	// show output
	cv::imwrite("assets/output/img_gvd_final.png", img_out);
	show_img("output", img_out);

}

inline void
geometry::gvd_graph2(const cv::Mat& img_gvd)
{
	// assert grayscale image
	if (img_gvd.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img, img_out;
	std::vector<line_t> vec_edges;
	std::vector<vertex_t> vec_vertices;

	// copy gvd
	img = img_gvd.clone();

	// create empty output
	cv::cvtColor(img, img_out, cv::COLOR_GRAY2BGR);
	// img_out.setTo(cv::Scalar(255, 255, 255));

	// -------------------------------------------------------------

	// lambdas

	auto find_expand_dir = [&](const cv::Point& pt, const std::vector<cv::Point>& vec_other_pts = {})
	{
		for (const auto& dir : PIXEL::DIR_VEC)
		{
			if (img.at<uchar>(pt + dir) == PIXEL::BLACK &&
				std::none_of(vec_other_pts.begin(), vec_other_pts.end(), [&](const auto& other_pt) { return (pt + dir == other_pt); })
			) return dir;
		}
	};

	auto is_v_shape = [&](const cv::Point& pt) -> std::optional<std::vector<cv::Point>>
	{
		std::vector<cv::Point> vec_pts;

		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_UP))
		{
			vec_pts.push_back(pt);
			vec_pts.push_back(pt + PIXEL::DIR_SE);
			vec_pts.push_back(pt + PIXEL::DIR_SW);
		}
		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_DOWN))
		{
			vec_pts.push_back(pt);
			vec_pts.push_back(pt + PIXEL::DIR_NE);
			vec_pts.push_back(pt + PIXEL::DIR_NW);
		}
		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_LEFT))
		{
			vec_pts.push_back(pt);
			vec_pts.push_back(pt + PIXEL::DIR_NE);
			vec_pts.push_back(pt + PIXEL::DIR_SE);
		}
		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_RIGHT))
		{
			vec_pts.push_back(pt);
			vec_pts.push_back(pt + PIXEL::DIR_NW);
			vec_pts.push_back(pt + PIXEL::DIR_SW);
		}
		
		return vec_pts.empty() ? std::nullopt : std::optional(vec_pts);
	};

	auto is_t_shape = [&](const cv::Point& pt)
	{
		;
	};

	auto is_asym_v_shape = [&](const cv::Point& pt)
	{
		;
	};

	auto is_diag = [&](const cv::Point& pt)
	{
		;
	};

	auto fix_asym_v_shapes = [&](const cv::Point& pt)
	{
		;
	};

	auto fix_t_shapes = [&](const cv::Point& pt)
	{
		;
	};

	auto expand_pt = [&](const cv::Point& pt, const cv::Point& dir) -> line_t
	{
		// expand point until white pixel occurs
		cv::Point pos_start = pt;
		cv::Point pos_end;
		cv::Point pos_cur = pos_start;

		while (pt_within_boundary(img, pos_cur + dir) && img.at<uchar>(pos_cur + dir) == PIXEL::BLACK)
		{
			// img.at<uchar>(pos_cur) = PIXEL::WHITE;
			pos_cur += dir;
		}

		// end point
		pos_end = pos_cur;

		// construct line
		return line_t{ pos_start, pos_end };
	};

	auto expand_v_shapes = [&]()
	{
		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (auto vec_pts = is_v_shape(pos))
			{
				for (const auto& pt : vec_pts.value())
				{
					// find expand dir; cannot be in dir of pts from vec_pts
					auto dir = find_expand_dir(pt, vec_pts.value());
					
					// expand point in specified dir
					auto edge = expand_pt(pt, dir);
					
					// append edge and vertices
					vec_vertices.push_back(edge.from);
					vec_vertices.push_back(edge.to);
					vec_edges.push_back(edge);

					cv::line(img_out, edge.from, edge.to, { 0, 0, 255 }, 1, 8);
				}
			}
		});
	};

	auto expand_diag = [&](const cv::Point& pt)
	{
		;
	};

	// -------------------------------------------------------------

	// perform thinning
	auto img_gvd_thin = thin_edges(img_gvd);
	show_img("gvd: thinned", img_gvd_thin);
	img = img_gvd_thin;

	// create output copy
	cv::cvtColor(img, img_out, cv::COLOR_GRAY2BGR);

	// expand v-shapes
	expand_v_shapes();

	// iterate_mat(img, [&](auto& pos, auto& pixel)
	// {
	// 	if (match_pattern_3x3(img, pos, { 0, 255, 255, 255, 0, 0, 0, 255, 255 }))
	// 		img_out.at<cv::Vec3b>(pos) = { 0, 0, 255 };
	// });

	cv::imwrite("assets/output/img_pattern_test.png", img_out);
	show_img("pattern test", img_out);

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
	cv::dilate(img_gvd_inv, img_gvd_inv, MORPH::STR_ELM_RECT_2x2);
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
geometry::test_gvd_houghlines()
{
	// variables
	cv::Mat img, img_gvd, img_edges, img_out;

	// big world
	img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	scale_img(img, DIM_BIGWORLD, SCALE_METER_PER_PX);

	// gvd using custom implementation
	img_gvd = gvd(img);
	show_img("gvd (custom)", img_gvd);

	// create empty output
	cv::cvtColor(img_gvd, img_out, cv::COLOR_GRAY2BGR);
	//img_out.setTo(cv::Scalar(255, 255, 255));

	// manual edges
	img_edges = img_gvd.clone();
	cv::bitwise_not(img_edges, img_edges);
	cv::erode(img_edges, img_edges, MORPH::STR_ELM_RECT_2x2);
	show_img("gvd: edges", img_edges);

	// hough lines configuration
	struct houghlines_config_t
	{
		// images
		cv::Mat img_gvd;
		cv::Mat img_edges;
		cv::Mat img_out;

		// variables
		const char*
		      wndw_name            = "gvd: hough lines";

		double   precision         = 1000;

		double   rho               = 0.1;
		int      rho_int           = rho * precision;
		double   theta             = CV_PI/45.0;
		int      theta_int         = theta * precision;
		int      th                = 1;

		double   get_rho()     { return (rho_int / precision); }
		double   get_theta()   { return (theta_int / precision); }

		int      max_double(double val) { return val * precision;}
	};

	// initialize config
	static houghlines_config_t hl_config;
	hl_config.img_gvd   = img_gvd.clone();
	hl_config.img_edges = img_edges.clone();
	hl_config.img_out   = img_out.clone();
	
	// lambda for callback
	auto on_trackbar = [](int val, void* userdata)
	{
		// get data
		auto data = (houghlines_config_t*)userdata;

		// clean image
		// data->img_out = data->img_gvd;
		cv::cvtColor(data->img_edges, data->img_out, cv::COLOR_GRAY2BGR);
		cv::bitwise_not(data->img_out, data->img_out);

		// perform standard hough line extraction
		std::vector<cv::Vec2f> vec_lines;
		cv::HoughLines(data->img_edges, vec_lines, data->get_rho(), data->get_theta(), data->th, 0, 0);

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

			cv::line(data->img_out, pt1, pt2, cv::Scalar(255, 0, 255), 1, 8);
		}

		cv::imshow(data->wndw_name, data->img_out);
	};

	// create window for sliders and output
	cv::namedWindow(hl_config.wndw_name, cv::WINDOW_AUTOSIZE);

	// add trackbars with callbacks
	cv::createTrackbar("rho", hl_config.wndw_name, &(hl_config.rho_int), hl_config.max_double(10), on_trackbar, (void*)(&hl_config));
	cv::createTrackbar("theta", hl_config.wndw_name, &(hl_config.theta_int), hl_config.max_double(CV_PI), on_trackbar, (void*)(&hl_config));
	cv::createTrackbar("threshold", hl_config.wndw_name, &(hl_config.th), 100, on_trackbar, (void*)(&hl_config));

	// call callback and display img
	on_trackbar(0, &hl_config);
	cv::waitKey(0);
}

inline void
geometry::test_gvd_houghlinesp()
{
	// variables
	cv::Mat img, img_gvd, img_edges, img_out;

	// big world
	img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	scale_img(img, DIM_BIGWORLD, SCALE_METER_PER_PX);

	// gvd using custom implementation
	img_gvd = gvd(img);
	show_img("gvd (custom)", img_gvd);

	// create empty output
	cv::cvtColor(img_gvd, img_out, cv::COLOR_GRAY2BGR);
	img_out.setTo(cv::Scalar(255, 255, 255));

	// manual edges
	img_edges = img_gvd.clone();
	cv::bitwise_not(img_edges, img_edges);
	cv::erode(img_edges, img_edges, MORPH::STR_ELM_RECT_2x2);
	show_img("gvd: edges", img_edges);

	// hough lines p configuration
	struct houghlinesp_config_t
	{
		// images
		cv::Mat img_edges;
		cv::Mat img_out;

		// variables
		const char*
		      wndw_name            = "gvd: hough lines p";

		double   precision         = 1000;

		double   rho               = 1.0;
		int      rho_int           = rho * precision;
		double   theta             = CV_PI/360.0;
		int      theta_int         = theta * precision;
		int      min_intersections = 1;
		double   min_pts           = 1;
		int      min_pts_int       = min_pts * precision;
		double   max_gap           = 1;
		int      max_gap_int       = max_gap * precision;

		double   get_rho()     { return (rho_int / precision); }
		double   get_theta()   { return (theta_int / precision); }
		double   get_min_pts() { return (min_pts_int / precision); }
		double   get_max_gap() { return (max_gap_int / precision); }

		int      max_double(double val) { return val * precision;}
	};

	// initialize config
	static houghlinesp_config_t hl_config;
	hl_config.img_edges = img_edges.clone();
	hl_config.img_out   = img_out.clone();

	// lambda for callback
	auto on_trackbar = [](int val, void* userdata)
	{
		// get data
		auto data = (houghlinesp_config_t*)userdata;

		// clean image
		data->img_out.setTo(cv::Scalar(255, 255, 255));

		// perform probabilistic hough line extraction
		std::vector<cv::Vec4i> vec_lines;
		cv::HoughLinesP(
			data->img_edges,           // input img (from edge detector)
			vec_lines,                 // ouput vector of [x_start, y_start, x_end, y_end] for each line
			data->get_rho(),           // resolution of parameter rho in pixels
			data->get_theta(),         // resolution of theta in radians
			data->min_intersections,   // minimum number of intersections to “detect” a line
			data->get_min_pts(),       // minimum number of points that can form a line
			data->get_max_gap()        // maximum gap between two points to be considered in the same line
		);

		// draw lines
		for (const auto& l : vec_lines)
			cv::line(data->img_out, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 255), 1, 8);

		cv::imshow(data->wndw_name, data->img_out);
	};

	// create window for sliders and output
	cv::namedWindow(hl_config.wndw_name, cv::WINDOW_AUTOSIZE);

	// add trackbars with callbacks
	cv::createTrackbar("rho", hl_config.wndw_name, &(hl_config.rho_int), hl_config.max_double(10), on_trackbar, (void*)(&hl_config));
	cv::createTrackbar("theta", hl_config.wndw_name, &(hl_config.theta_int), hl_config.max_double(CV_PI), on_trackbar, (void*)(&hl_config));
	cv::createTrackbar("min_intersections", hl_config.wndw_name, &(hl_config.min_intersections), 100, on_trackbar, (void*)(&hl_config));
	cv::createTrackbar("min_pts", hl_config.wndw_name, &(hl_config.min_pts_int), hl_config.max_double(100), on_trackbar, (void*)(&hl_config));
	cv::createTrackbar("max_gap", hl_config.wndw_name, &(hl_config.max_gap_int), hl_config.max_double(100), on_trackbar, (void*)(&hl_config));

	// call callback and display img
	on_trackbar(0, &hl_config);
	cv::waitKey(0);
}

inline void
geometry::test_brushfire_and_gvd()
{
	// load and scale image

	// small world
	//auto img = load_img(PATH_IMG_SMALLWORLD, cv::IMREAD_GRAYSCALE);
	//scale_img(img, DIM_SMALLWORLD, SCALE_METER_PER_PX);

	// big world: load, add black border and resize
	auto img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	add_border(img, IMG_BORDER_SIZE, cv::Scalar(0));
	scale_img(img, DIM_BIGWORLD, SCALE_METER_PER_PX);
	// cv::resize(img, img, cv::Size(846, 564), 0.0, 0.0, cv::INTER_NEAREST);

	// show map
	// show_img("map", img);

	// detecting GVD edges using HoughLinesP w/ trackbars
	// test_gvd_houghlinesp();

	// detecting GVD edges using HoughLines overlaying method
	// test_gvd_houghlines();

	// brushfire
	auto img_bf = brushfire(img);
	// show_img("brushfire", img_bf);

	// gvd using brushfire
	auto img_gvdbf = gvd_bf(img, img_bf);
	// show_img("gvd (bf)", img_gvdbf);

	// gvd using custom implementation
	// auto img_gvd = gvd(img);
	// show_img("gvd (custom)", img_gvd);

	// make graph from GVD image
	gvd_graph2(img_gvdbf);

	// opencv GVD
	// auto img_gvdcv = gvd_opencv(img);
	// show_img("gvd (opencv)", img_gvdcv);
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