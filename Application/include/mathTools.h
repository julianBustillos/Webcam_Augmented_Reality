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
	static float pointPointDistance(cv::Vec2i p1, cv::Vec2i p2);
	static int grayScaleValue(const cv::Mat & frame, cv::Vec2i point);
	static cv::Vec2i averagePoint(cv::Vec2i p1, cv::Vec2i p2);
	static cv::Vec2i linesIntersection(cv::Vec2i l1p1, cv::Vec2i l1p2, cv::Vec2i l2p1, cv::Vec2i l2p2);
	static float mod2Pi(float val);
	static double diffSquareNorm(cv::Vec3d vec1, cv::Vec3d vec2);
	static double det3x3(const cv::Mat & mat);
	static double norm3x1(cv::Mat vec);
	static void findHomography(const std::vector<cv::Vec2d> & original, const std::vector<cv::Vec2d> & transformed, cv::Mat & homography);
	static void findHomography(const std::vector<cv::Vec2i> & original, const std::vector<cv::Vec2d> & transformed, cv::Mat & homography);
};
