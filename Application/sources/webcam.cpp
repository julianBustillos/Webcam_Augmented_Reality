#include "webcam.h"
#include <iostream>


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

int Webcam::getWidth() const
{
	return (int)capture.get(cv::CAP_PROP_FRAME_WIDTH);
}

int Webcam::getHeight() const
{
	return (int)capture.get(cv::CAP_PROP_FRAME_HEIGHT);
}

void Webcam::read()
{
	cv::Mat temp;
	capture.read(temp);
	if (temp.empty()) {
		std::cerr << "ERROR! blank frame grabbed" << std::endl;
	}
	cv::flip(temp, frame, 1);
}

cv::Mat & Webcam::getFrame()
{
	return frame;
}
