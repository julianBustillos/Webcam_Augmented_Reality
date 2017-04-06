#pragma once

#include <opencv2/opencv.hpp>

class Webcam {
public:
	Webcam();
	~Webcam();
	int getWidth() const;
	int getHeight() const;
	void read();
	cv::Mat & getFrame();

private:
	cv::Mat temp;
	cv::Mat frame;
	cv::VideoCapture capture;
};