#pragma once

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <time.h>
#include "cornerDetector.h"


class MathTools {
public:
	static int convolution(const cv::Mat & frame, const cv::Vec2i & frameSize, const std::vector<int> & filter, const cv::Vec2i & position, const cv::Vec2i & scanDir, int channel);
	static int random(int sup);
	static float orientationDiff(float or1, float or2);
	static float edgelOrientation(int scanDirVal, int strideDirVal, EdgelType type);
	static float lineOrientation(cv::Vec2i lp1, cv::Vec2i lp2);
	static float pointLineDistance(cv::Vec2i lp1, cv::Vec2i lp2, cv::Vec2i p);
	static float poinPointDistance(cv::Vec2i p1, cv::Vec2i p2);
};
