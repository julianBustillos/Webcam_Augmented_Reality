#include <iostream>
#include "webcam.h"
#include "GLManager.h"


int main(int argc, char *argv[])
{


	// Initialization
	Webcam webcam;
	GLManager glManager(webcam.getFrame());
	std::cout << "INITIALIZATION ENDED" << std::endl << std::endl;

	// Main loop
	while (glManager.running()) {
		glManager.event();
		webcam.read();
		glManager.drawFrame(webcam.getFrame());
		glManager.swapBuffers();
	}

	return EXIT_SUCCESS;
}