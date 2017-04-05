#include "debug.h"
#include <iostream>
#include "webcam.h"
#include "GLManager.h"
#include <constants.h>
#include "frameProcessing.h"
#include <stdlib.h>
#include <time.h>

#ifdef DEBUG
#include "debugInfo.h"
DebugInfo info;
#endif


int main(int argc, char *argv[])
{
	// Initialization
	std::cout << "INITIALIZATION..." << std::endl;
	std::srand((int)time(NULL));
	Webcam webcam;
	GLManager glManager(webcam.getFrame());
	FrameProcessing processing(webcam.getWidth(), webcam.getHeight());
	std::cout << "INITIALIZATION ENDED" << std::endl << std::endl;

	// Main loop
	while (glManager.running()) {
		glManager.event();
		webcam.read();
		processing.execute(webcam.getFrame());
#ifdef DEBUG
		info.printOnFrame(webcam.getFrame(), processing);
#endif
		glManager.drawFrame(webcam.getFrame());
		glManager.swapBuffers();
	}

	return EXIT_SUCCESS;
}