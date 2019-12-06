#include "modules/core.h"

int main(int argc, char** argv) 
{
	// initiailize the system
	core::init(argc, argv);

	// tests
	// if (TEST_GVD)           geometry::test_brushfire_and_gvd();
	if (TEST_GVD)           roadmap::test();
	if (TEST_LINE_SEG)      geometry::test_segment_intersect();
	if (TEST_MARBLE_DETECT) core::test_run("Marble_data_big_world2.avi");
	if (TEST_EXIT_AFTER)    return 0;

	// operate until ESC key pressed
	core::run();
	
	// exit
	return 0;
}