#include <iostream>
#include "GLManager.h"

int main(int argc, char *argv[])
{
	GLManager glManager;

	// Initialization
	glManager.init(800, 600);

	// Main loop
	while (glManager.running()) {
		glManager.event();
		glManager.swapBuffers();
	}

	return EXIT_SUCCESS;
}