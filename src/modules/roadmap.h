# pragma once

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
// declarations for ::roadmap
// --------------------------------------------------------------------------------

namespace roadmap
{

	// main methods

	cv::Mat
	brushfire(const cv::Mat& img_map, bool normalize = false);

	cv::Mat
	gvd_img(const cv::Mat& img_map, cv::Mat img_bf);

	cv::Mat
	gvd_graph(const cv::Mat& img_map, const cv::Mat& img_gvd);

	void
	test();

	// helper methods

}

// --------------------------------------------------------------------------------
// definitions for ::roadmap
// --------------------------------------------------------------------------------

inline cv::Mat
roadmap::brushfire(const cv::Mat& img_map, bool normalize)
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

	// method for checking whether 3x3 ROI contains value n (8-adj)
	auto eight_adj_contains = [&](const cv::Point& pt, uchar value)
	{
		size_t count = 0;

		iterate_8adj(img, pt, [&](auto& pos, auto& pixel) {
			count += (size_t)(pixel == value);
		});

		return count > 0;
	};

	// brushfire algorithm
	// iterate all 0-valued pixels and increment n each (complete) iteration
	// if zero-th pixel has the value n within its adjacency, then set that pixel to n + 1
	for (size_t n = 1; num_zero_px != 0 ; ++n)
	{
		if (n > 255)
			throw std::runtime_error("Value of n has exceeded 255.");

		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (pixel == 0 && eight_adj_contains(pos, n))
			{
				pixel = n + 1;
				--num_zero_px;
			}
		});
	}

	// normalize brushfire
	if (normalize)
	{
		img.convertTo(img, CV_32F);
		cv::normalize(img, img, 0, 1, cv::NORM_MINMAX);
	}

	return img;
}

inline cv::Mat
roadmap::gvd_img(const cv::Mat& img_map, cv::Mat img_bf)
{
	// assert grayscale image
	if (img_map.channels() != 1)
		throw std::runtime_error(ERR_IMG_NOT_GRAY);

	// variables
	cv::Mat img_map_inv, img_laplace, img_sobel, img_sobel_abs, img_gvd;

	// compute inverse
	cv::bitwise_not(img_map, img_map_inv);

	// compute brushfire if not specified
	if (img_bf.empty())
		img_bf = brushfire(img_map);

	// use Sobel operator to find gradient
	cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y;

	// gradient X
	cv::Sobel(img_bf, grad_x, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	// cv::Scharr(img_bf, grad_x, CV_32F, 1, 0, 1, 0, cv::BORDER_DEFAULT);

	// gradient Y
	cv::Sobel(img_bf, grad_y, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
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
roadmap::gvd_graph(const cv::Mat& img_map, const cv::Mat& img_gvd)
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

	// create colorized output
	cv::cvtColor(img, img_out, cv::COLOR_GRAY2BGR);

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

	auto is_v_shape = [&](const cv::Point& pt) -> std::tuple<cv::Point, std::vector<cv::Point>>
	{
		// return direction and vector of expandable node if match found

		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_UP))
			return { PIXEL::DIR_N, std::vector<cv::Point>{ pt, pt + PIXEL::DIR_SE, pt + PIXEL::DIR_SW } };

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_DOWN))
			return { PIXEL::DIR_S, std::vector<cv::Point>{ pt, pt + PIXEL::DIR_NE, pt + PIXEL::DIR_NW } };

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_LEFT))
			return { PIXEL::DIR_W, std::vector<cv::Point>{ pt, pt + PIXEL::DIR_NE, pt + PIXEL::DIR_SE } };

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_V_RIGHT))
			return { PIXEL::DIR_E, std::vector<cv::Point>{ pt, pt + PIXEL::DIR_NW, pt + PIXEL::DIR_SW } };

		// no match
		return { PIXEL::DIR_NONE, {} };
	};

	auto is_t_shape = [&](const cv::Point& pt) -> std::optional<cv::Point>
	{

		if (match_pattern_3x3(img, pt, PIXEL::PAT_T_UP))
			return PIXEL::DIR_N;

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_T_DOWN))
			return PIXEL::DIR_S;

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_T_LEFT))
			return PIXEL::DIR_W;

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_T_RIGHT))
			return PIXEL::DIR_E;

		// no match
		else
			return std::nullopt;
	};

	auto is_asym_v_shape = [&](const cv::Point& pt) -> std::optional<cv::Mat>
	{
		for (const auto& p : PIXEL::PAT_ASYM_V_VEC)
			if (match_pattern_3x3(img, pt, p))
				return p;

		return std::nullopt;
	};

	auto is_cross_shape = [&](const cv::Point& pt) -> std::optional<std::vector<cv::Point>>
	{
		if (match_pattern_3x3(img, pt, PIXEL::PAT_CROSS))
			return std::vector<cv::Point>{ PIXEL::DIR_N, PIXEL::DIR_S, PIXEL::DIR_E, PIXEL::DIR_W };

		else
			return std::nullopt;
	};

	auto is_diag = [&](const cv::Point& pt) -> std::optional<cv::Point>
	{
		// boundary check
		if (not pt_within_boundary(img, pt))
			return std::nullopt;

		if (match_pattern_3x3(img, pt, PIXEL::PAT_DIAG_NW))
			return PIXEL::DIR_NW;

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_DIAG_NE))
			return PIXEL::DIR_NE;

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_DIAG_SW))
			return PIXEL::DIR_SW;

		else
		if (match_pattern_3x3(img, pt, PIXEL::PAT_DIAG_SE))
			return PIXEL::DIR_SE;
		else
			return std::nullopt;
	};

	auto fix_asym_v_shapes = [&]()
	{
		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			cv::Mat fixer_pattern, roi;

			if (auto pattern = is_asym_v_shape(pos))
			{
				// determine appropriate fixer pattern

				// default shape
				fixer_pattern = PIXEL::PAT_EMPTY;

				// asymmetric v-shape pointing either up or left
				if (mat_equal(pattern.value(), PIXEL::PAT_ASYM_V_UP_LEFT))
				{
					// v-shape must be upward
					if (img.at<uchar>(pos + PIXEL::DIR_SE + PIXEL::DIR_W) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_UP;

					// v-shape must be leftward
					if (img.at<uchar>(pos + PIXEL::DIR_NE + PIXEL::DIR_N) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_LEFT;
				}

				// asymmetric v-shape pointing either up or right
				else
				if (mat_equal(pattern.value(), PIXEL::PAT_ASYM_V_UP_RIGHT))
				{
					// v-shape must be upward
					if (img.at<uchar>(pos + PIXEL::DIR_SE + PIXEL::DIR_E) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_UP;

					// v-shape must be rightward
					if (img.at<uchar>(pos + PIXEL::DIR_NW + PIXEL::DIR_N) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_RIGHT;
				}

				// asymmetric v-shape pointing either down or left
				else
				if (mat_equal(pattern.value(), PIXEL::PAT_ASYM_V_DOWN_LEFT))
				{
					// v-shape must be downward
					if (img.at<uchar>(pos + PIXEL::DIR_NW + PIXEL::DIR_W) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_DOWN;

					// v-shape must be rightward
					if (img.at<uchar>(pos + PIXEL::DIR_SE + PIXEL::DIR_S) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_LEFT;
				}

				// asymmetric v-shape pointing either down or right (darius)
				else
				if (mat_equal(pattern.value(), PIXEL::PAT_ASYM_V_DOWN_RIGHT))
				{
					// v-shape must be downward
					if (img.at<uchar>(pos + PIXEL::DIR_NE + PIXEL::DIR_E) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_DOWN;

					// v-shape must be rightward
					if (img.at<uchar>(pos + PIXEL::DIR_SW + PIXEL::DIR_S) == PIXEL::BLACK)
						fixer_pattern = PIXEL::PAT_V_RIGHT;
				}

				// something went wrong
				else
					throw std::runtime_error(ERR_FIX_ASYM_V_SHAPE);
				
				// region of interest
				roi = img(cv::Rect(pos.x - 1, pos.y - 1, 3, 3));
				
				// debug
				// std::cout << pos << std::endl;
				// std::cout << "ROI (" << roi.size() << ")\n" << roi << std::endl;
				// std::cout << "PAT (" << pattern.value().size() << ")\n" << pattern.value() << std::endl;
				// std::cout << "FIX (" << fixer_pattern.size() << ")\n" << fixer_pattern << std::endl;

				// apply the derived fixer pattern to ROI
				cv::bitwise_or(roi, fixer_pattern, roi);
				cv::bitwise_and(roi, fixer_pattern, roi);
			}
		});
	};

	auto fix_t_shapes = [&]()
	{
		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (auto dir = is_t_shape(pos))
				img.at<uchar>(pos) = PIXEL::WHITE;
		});
	};

	auto fix_cross_shapes = [&]()
	{
		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (is_cross_shape(pos))
				img.at<uchar>(pos) = PIXEL::WHITE;
		});
	};

	auto expand_pt = [](cv::Mat& img, const cv::Point& pt, const cv::Point& dir) -> line_t
	{
		// expand point until white pixel occurs
		cv::Point pos_start = pt;
		cv::Point pos_end;
		cv::Point pos_cur = pos_start;

		while (pt_within_boundary(img, pos_cur + dir) && img.at<uchar>(pos_cur + dir) == PIXEL::BLACK)
		{
			img.at<uchar>(pos_cur) = PIXEL::WHITE;
			pos_cur += dir;
		}

		// end point
		pos_end = pos_cur;
		img.at<uchar>(pos_end) = PIXEL::WHITE;

		// construct line
		return line_t{ pos_start, pos_end };
	};

	auto expand_node = [&](const cv::Point& pt, const cv::Point& dir, const std::vector<cv::Point>& vec_pts_self)
	{
		// expand point until black pixel occurs
		cv::Point pos_start = pt;
		cv::Point pos_cur   = pos_start;

		auto img_temp = img.clone();

		while (pt_within_boundary(img, pos_cur + dir) && img.at<uchar>(pos_cur + dir) == PIXEL::WHITE)
		{
			pos_cur += dir;
			img_temp.at<uchar>(pos_cur) = PIXEL::BLACK;

			// if endpoint hits an obstacle on map, discard changes
			if (img_map.at<uchar>(pos_cur) == PIXEL::BLACK)
				return;
		}

		// if endpoint connects to same blob as self, discard changes
		if (std::any_of(vec_pts_self.begin(), vec_pts_self.end(), [&](const auto& pt_self){
			return (pos_cur == pt_self || pos_cur + dir == pt_self);
		})) return;

		img = img_temp;
	};

	auto connect_all_networks = [&]()
	{
		// find all 8-adj connected components (blos) sorted by size
		auto vec_blobs = connected_blobs(img, 8, true);

		// expand all blobs; skip the first being the largest network
		for (size_t i = 1; i < vec_blobs.size(); i++)
		{
			auto& vec_pts = vec_blobs[i].vec_pts;

			for (const auto& pt : vec_pts)
			{
				if (auto [dir, vec_nodes] = is_v_shape(pt); dir != PIXEL::DIR_NONE)
					expand_node(pt, -dir, vec_pts);

				if (auto dir = is_t_shape(pt))
					expand_node(pt, -dir.value(), vec_pts);
			}
		}
	};

	auto remove_disjoint_networks = [&]()
	{
		auto vec_blobs = connected_blobs(img, 8, true);
		img.setTo(PIXEL::WHITE);
		for (const auto& pt : vec_blobs[0].vec_pts)
			img.at<uchar>(pt) = PIXEL::BLACK;
	};

	auto expand_cross_shapes = [&]()
	{
		// expansion must be performed on copy
		// such that the white pixels are only removed after all iterations
		auto img_temp = img.clone();

		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (auto vec_dir = is_cross_shape(pos))
			{
				for (const auto& dir : *vec_dir)
				{					
					// expand point in specified dir
					auto edge = expand_pt(img_temp, pos, dir);
					
					// append edge and vertices
					vec_vertices.push_back(edge.from);
					vec_vertices.push_back(edge.to);
					vec_edges.push_back(edge);

					cv::line(img_out, edge.from, edge.to, { 0, 0, 255 }, 1, 8);
				}
			}
		});

		// apply changes
		img = img_temp;
	};

	auto expand_v_shapes = [&]()
	{
		// expansion must be performed on copy
		// such that the white pixels are only removed after all iterations
		auto img_temp = img.clone();

		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (auto [dir, vec_nodes] = is_v_shape(pos); dir != PIXEL::DIR_NONE)
			{
				for (const auto& pt : vec_nodes)
				{
					// find expand dir; cannot be in dir of pts from vec_pts
					auto dir = find_expand_dir(pt, vec_nodes);
					
					// expand point in specified dir
					auto edge = expand_pt(img_temp, pt, dir);
					
					// append edge and vertices
					vec_vertices.push_back(edge.from);
					vec_vertices.push_back(edge.to);
					vec_edges.push_back(edge);

					cv::line(img_out, edge.from, edge.to, { 0, 0, 255 }, 1, 8);
				}
			}
		});

		// apply changes
		img = img_temp;
	};

	auto expand_diag = [&]()
	{
		iterate_mat(img, [&](auto& pos, auto& pixel)
		{
			if (auto dir = is_diag(pos))
			{
				// expand point in specified dir
				auto edge = expand_pt(img, pos, dir.value());
				
				// append edge and vertices
				vec_vertices.push_back(edge.from);
				vec_vertices.push_back(edge.to);
				vec_edges.push_back(edge);

				cv::line(img_out, edge.from, edge.to, { 0, 0, 255 }, 1, 8);
			}
		});
	};

	auto find_vertices = [&]()
	{
		vec_vertices.clear();

		for (const auto& l : vec_edges)
		{
			bool found_v_from = false;
			bool found_v_to   = false;

			iterate_8adj(img, l.from, [&](auto& pos, auto& pixel)
			{
				if (std::any_of(vec_vertices.begin(), vec_vertices.end(), [&](const auto& pt){
					return pos == pt;
				}))	found_v_from = true;
			});

			iterate_8adj(img, l.to, [&](auto& pos, auto& pixel)
			{
				if (std::any_of(vec_vertices.begin(), vec_vertices.end(), [&](const auto& pt){
					return pos == pt;
				}))	found_v_to = true;
			});

			if (not found_v_from)
				vec_vertices.push_back(l.from);

			if (not found_v_to)
				vec_vertices.push_back(l.to);
		}
	};

	// -------------------------------------------------------------

	// perform thinning
	img = thin_edges(img_gvd);
	show_img("gvd: thin lines (step 1)", img);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_step1.png", img);

	// connect all networks and remove any remaining disjoint networks (closed rooms)
	connect_all_networks();
	remove_disjoint_networks();
	show_img("gvd: connect all networks + remove disjoint (step 2)", img);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_step2.png", img);

	// fix cross shapes
	fix_cross_shapes();
	show_img("gvd: fix cross shapes (step 3)", img);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_step3.png", img);

	// fix asymmetric V-shapes
	fix_asym_v_shapes();
	show_img("gvd: fix asym v shapes (step 4)", img);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_step4.png", img);

	// fix T-shapes
	fix_t_shapes();
	show_img("gvd: fix t shapes (step 5)", img);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_step5.png", img);

	// create output copy
	cv::cvtColor(img, img_out, cv::COLOR_GRAY2BGR);

	// expand v-shapes
	expand_v_shapes();
	show_img("gvd: expand v shapes (step 6)", img);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_step6.png", img);

	// expand diagonals
	expand_diag();
	show_img("gvd: expand diagonals (step 7)", img);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_step7.png", img);

	// draw vertices
	find_vertices();
	for (const auto& v : vec_vertices)
		img_out.at<cv::Vec3b>(v) = { 255, 0, 0 };

	// number of edges found
	// std::cout << "number of edges: " << vec_edges.size() << std::endl;
	// std::cout << "number of duplicate edges: " << count_duplicate_lines(vec_edges) << std::endl;

	return img_out;
}

inline void
roadmap::test()
{
	// big world: load, add black border and resize
	auto img_map = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	add_border(img_map, IMG_BORDER_SIZE, cv::Scalar(0));
	scale_img(img_map, DIM_BIGWORLD, SCALE_METER_PER_PX);

	// show map
	show_img("map", img_map);

	// brushfire (normalized)
	auto img_bf = brushfire(img_map, true);
	show_img("brushfire (normalized)", img_bf);

	// GVD from brushfire (image only)
	auto img_gvdbf = gvd_img(img_map, img_bf);
	show_img("gvd (bf)", img_gvdbf);

	// graph from GVD (edges and vertices)
	auto img_gvd = gvd_graph(img_map, img_gvdbf);

	// final image
	combine_img(img_gvd, img_map);
	cv::imwrite(PATH_IMG_GVD_OUTPUT + "img_gvd_final.png", img_gvd);
	show_img("gvd (final)", img_gvd);
}