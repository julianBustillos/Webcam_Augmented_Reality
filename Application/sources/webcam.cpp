#include "webcam.h"
#include <iostream>
#include "debug.h"

#ifdef DEBUG
#include "debugInfo.h"
extern DebugInfo info;
#endif

Webcam::Webcam()
{
	capture.open(0);
	if (!capture.isOpened()) {
		std::cerr << "ERROR! Unable to open camera" << std::endl;
	}
	else {
		std::cout << "Webcam successfully initialized !" << std::endl;
	}
	read();
}

Webcam::~Webcam()
{
}

int Webcam::getWidth() 
{
	return (int)capture.get(cv::CAP_PROP_FRAME_WIDTH);
}

int Webcam::getHeight() 
{
	return (int)capture.get(cv::CAP_PROP_FRAME_HEIGHT);
}

void Webcam::read()
{
#ifndef DEBUG
	capture.read(temp);
#else 
	if (!info.isPaused()) {
		capture.read(temp);
	}
#endif

	if (temp.empty()) {
		std::cerr << "ERROR! blank frame grabbed" << std::endl;
	}
	cv::flip(temp, frame, 1);
}

cv::Mat & Webcam::getFrame()
{
	return frame;
}
