#include "modules/core.h"

int main(int argc, char** argv) 
{
	// initiailize the system
	core::init(argc, argv);

	// tests
	if (TEST_GVD)           brushfire();
	if (TEST_LINE_SEG)      geometry::test_segment_intersect();
	if (TEST_CELL_DECOMP)   pathgen::test_cell_decomp();
	if (TEST_MARBLE_DETECT) core::test_run("Marble_data_big_world2.avi");

	// operate until ESC key pressed
	core::run();
	
	// exit
	return 0;
}