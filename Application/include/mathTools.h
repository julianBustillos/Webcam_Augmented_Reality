#pragma once

#include <opencv2/opencv.hpp>


class MathTools {
public:
	static int convolution(cv::Mat & frame, cv::Vec2i frameSize, std::vector<int> filter, cv::Vec2i position, cv::Vec2i scanDir, int channel);
};
