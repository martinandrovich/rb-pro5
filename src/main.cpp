#include "modules/core.h"

const int test_on = 1;

int main(int argc, char** argv) 
{
	// initiailize the system
	core::init(argc, argv);

	if(test_on) core::test_run("MarbleRun3.avi");
	// operate until ESC key pressed
	core::run();
	
	// exit
	return 0;
}