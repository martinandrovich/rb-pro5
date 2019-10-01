#include "modules/core.h"

int main(int argc, char** argv) 
{
	// initiailize the system
	core::init();

	// operate until ESC key pressed
	core::run();
	
	// exit
	return 0;
}