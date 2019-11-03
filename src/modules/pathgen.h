#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// --------------------------------------------------------------------------------
// helpers for ::pathgen | will be relocated
// --------------------------------------------------------------------------------

struct adj_graph_t
{

};

// --------------------------------------------------------------------------------
// declarations for ::pathgen
// --------------------------------------------------------------------------------

namespace pathgen
{
	
	// public methods

	adj_graph_t
	gen_adj_graph(const cv::Mat& img);

	void
	test_cell_decomp();

	// private methods

	static void
	cell_decomp();
}

// --------------------------------------------------------------------------------
// method defintions for ::pathgen
// --------------------------------------------------------------------------------

// -- public ----------------------------------------------------------------------

inline adj_graph_t
pathgen::gen_adj_graph(const cv::Mat& img)
{
	// locate map (file)
	;

	// extract edges and vertices
	;

	// trapezoidal cell decomoposition
	;

	// return graph
	;
}

inline void
pathgen::test_cell_decomp()
{
	return;
}

// -- private ---------------------------------------------------------------------

static void
pathgen::cell_decomp()
{
	return;
}