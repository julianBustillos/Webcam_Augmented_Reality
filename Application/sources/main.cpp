#include <iostream>
#include "webcam.h"
#include "GLManager.h"
#include <macros.h>
#include "debugInfo.h"

DebugInfo *info = nullptr;


int main(int argc, char *argv[])
{


	// Initialization
	if (DEBUG) {
		info = new DebugInfo();
	}
	Webcam webcam;
	GLManager glManager(webcam.getFrame());
	std::cout << "INITIALIZATION ENDED" << std::endl << std::endl;

	// Main loop
	while (glManager.running()) {
		glManager.event();
		webcam.read();
		if (info) {
			info->printOnFrame(webcam.getFrame());
		}
		glManager.drawFrame(webcam.getFrame());
		glManager.swapBuffers();
	}

	return EXIT_SUCCESS;
}