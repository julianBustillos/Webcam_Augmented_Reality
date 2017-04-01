#pragma once

#include <opencv2/opencv.hpp>

class Webcam {
public:
	Webcam();
	~Webcam();
	int getWidth();
	int getHeight();
	void read();
	cv::Mat & getFrame();

private:
	cv::Mat frame;
	cv::VideoCapture capture;
};