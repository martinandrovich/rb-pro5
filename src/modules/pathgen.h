#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../constants.h"

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
	
	// load image
	auto img = cv::imread(PATH_IMG_ENVIRON_DEMO, cv::IMREAD_GRAYSCALE);

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
