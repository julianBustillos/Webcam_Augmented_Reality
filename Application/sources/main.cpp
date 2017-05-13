#include "debug.h"
#include <iostream>
#include "webcam.h"
#include "GLManager.h"
#include <constants.h>
#include "cornerDetector.h"
#include "markerRecognizer.h"
#include <stdlib.h>
#include <time.h>
#include "PnPSolver.h"

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
	MarkerRecognizer recognizer(webcam.getWidth(), webcam.getHeight());
	PnPSolver pnp(webcam.getWidth(), webcam.getHeight());
	std::cout << "INITIALIZATION ENDED" << std::endl << std::endl;

	// Main loop
	while (glManager.running()) {
		glManager.event();
		webcam.read();
#ifdef DEBUG
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
#endif
		detector.setROI(recognizer.getROI());
		detector.execute(webcam.getFrame());
		recognizer.searchMarker(webcam.getFrame(), detector.getCornerGroupsList());
		pnp.solve(recognizer.getOrderedCorners(), recognizer.identified());
#ifdef DEBUG
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		double time = std::chrono::duration<double>(end - start).count();
		info.printOnFrame(webcam.getFrame(), time, detector, recognizer);

		if (recognizer.identified() && info.showMesh()) {
#else
		if (recognizer.identified()) {
#endif
			glManager.draw(webcam.getFrame(), &pnp);
		}
		else {
			glManager.draw(webcam.getFrame(), nullptr);
		}
		glManager.swapBuffers();
	}

	return EXIT_SUCCESS;
}