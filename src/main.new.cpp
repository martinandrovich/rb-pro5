#include "modules/core.h"

int main(int argc, char** argv) 
{
	// initiailize the system
	core::init(argc, argv);

	// operate until ESC key pressed
	core::run();
	
	// exit
	return 0;
}