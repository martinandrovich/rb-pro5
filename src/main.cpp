#include "modules/core.h"

int main(int argc, char** argv) 
{
	// initiailize the system
	core::init(argc, argv);

	// tests
	if(TEST_CELL_DECOMP) pathgen::test_cell_decomp();
	if(TEST_MARBLE_DETECT) core::test_run("MarbleRun3.avi");

	// operate until ESC key pressed
	core::run();
	
	// exit
	return 0;
}