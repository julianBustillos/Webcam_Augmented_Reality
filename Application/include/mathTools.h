#pragma once

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>


class MathTools {
public:
	static int convolution(cv::Mat & frame, cv::Vec2i frameSize, std::vector<int> filter, cv::Vec2i position, cv::Vec2i scanDir, int channel);
	static int random(int sup);
	static float orientationDiff(float or1, float or2);
	static float lineOrientations(cv::Vec2i lp1, cv::Vec2i lp2);
	static float pointLineDistance(cv::Vec2i lp1, cv::Vec2i lp2, cv::Vec2i p);
};
