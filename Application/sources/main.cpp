#include <iostream>
#include "webcam.h"
#include "GLManager.h"
#include <macros.h>
#include "debugInfo.h"
#include "frameProcessing.h"
#include <stdlib.h>
#include <time.h>

DebugInfo *info = nullptr;


int main(int argc, char *argv[])
{


	// Initialization
	std::srand((int)time(NULL));
	if (DEBUG) {
		info = new DebugInfo();
	}
	Webcam webcam;
	GLManager glManager(webcam.getFrame());
	FrameProcessing processing(webcam.getWidth(), webcam.getHeight());
	std::cout << "INITIALIZATION ENDED" << std::endl << std::endl;

	// Main loop
	while (glManager.running()) {
		glManager.event();
		webcam.read();
		processing.execute(webcam.getFrame());
		if (info) {
			info->printOnFrame(webcam.getFrame(), processing);
		}
		glManager.drawFrame(webcam.getFrame());
		glManager.swapBuffers();
	}

	return EXIT_SUCCESS;
}