#include "debug.h"
#include <iostream>
#include "webcam.h"
#include "GLManager.h"
#include <constants.h>
#include "cornerDetector.h"
#include "markerRecognizer.h"

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
	CornerDetector detector(webcam.getWidth(), webcam.getHeight());
	MarkerRecognizer recognizer;
	std::cout << "INITIALIZATION ENDED" << std::endl << std::endl;

	// Main loop
	while (glManager.running()) {
		glManager.event();
		webcam.read();
		detector.execute(webcam.getFrame());
		recognizer.searchMarker(webcam.getFrame(), detector.getCornerGroupsList());
#ifdef DEBUG
		info.printOnFrame(webcam.getFrame(), detector);
#endif
		glManager.drawFrame(webcam.getFrame());
		glManager.swapBuffers();
	}

	return EXIT_SUCCESS;
}