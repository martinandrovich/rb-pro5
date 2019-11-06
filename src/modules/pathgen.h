#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../constants.h"
#include "utils.h"

#include "types/adj_graph.h"

// --------------------------------------------------------------------------------
// declarations for ::pathgen
// --------------------------------------------------------------------------------

namespace pathgen
{
	
	// public methods
	
	void
	test_cell_decomp();

	// private methods

	static void
	_cell_decomp();

}

// --------------------------------------------------------------------------------
// method defintions for ::pathgen
// --------------------------------------------------------------------------------

// -- public ----------------------------------------------------------------------

inline void
pathgen::test_cell_decomp()
{

	// determine load paths from current world data
	// ---
	
	// load and scale image
	auto img = load_img(PATH_IMG_BIGWORLD, cv::IMREAD_GRAYSCALE);
	scale_image(img, DIM_BIGWORLD, SCALE_METER_PER_PX);

	// generate adjacency graph
	auto adj_graph = adj_graph_t::gen_adj_graph(img);

	return;
}

// -- private ---------------------------------------------------------------------

static void
pathgen::_cell_decomp()
{
	std::cout << "private cell decomposition" << std::endl;
	return;
}
