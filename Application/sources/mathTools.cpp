#include "mathTools.h"
#include "constants.h"


int MathTools::convolution(const cv::Mat & frame, const cv::Vec2i & frameSize, const std::vector<int> & filter, const cv::Vec2i & position, const cv::Vec2i & scanDir, int channel)
{
	int offset = (int)floor(filter.size() / 2.0f);
	int sum = 0;
	int firstPos = position.dot(scanDir) - offset;
	int maxPos = frameSize.dot(scanDir);

	if (firstPos < 0 || firstPos + filter.size() - 1 >= maxPos) {
		return 0;
	}

	for (int k = 0; k < filter.size(); k++) {
		cv::Vec2i currentPos = position + (k - offset) * scanDir;
		sum += frame.at<cv::Vec3b>(currentPos[0], currentPos[1])[channel] * filter[filter.size() - k - 1];
	}

	return sum;
}

int MathTools::random(int sup)
{
	return std::rand() % sup;
}

float MathTools::orientationDiff(float or1, float or2)
{
	float diff1;
	float diff2;

	diff1 = abs(or1 - or2);
	if (or1 < or2) {
		diff2 = abs(or1 + 2 * M_PI - or2);
	}
	else {
		diff2 = abs(or2 + 2 * M_PI - or1);
	}

	return std::min(diff1, diff2);
}

float MathTools::edgelOrientation(int scanDirVal, int strideDirVal, EdgelType type)
{
	float orientation;
	if (type == EdgelType::vertical) {
		orientation = std::atan2f((float)scanDirVal, (float)strideDirVal);
	}
	else {
		orientation = std::atan2f((float)strideDirVal, (float)scanDirVal);
	}
	return orientation;
}

int MathTools::grayScaleValue(const cv::Mat & frame, cv::Vec2i point)
{
	int B = frame.at<cv::Vec3b>(point)[0];
	int G = frame.at<cv::Vec3b>(point)[1];
	int R = frame.at<cv::Vec3b>(point)[2];

	return (int)(0.0771f * B + 0.7154f * G + 0.2125f * R);
}

cv::Vec2i MathTools::averagePoint(cv::Vec2i p1, cv::Vec2i p2)
{
	cv::Vec2f average = (p1 + p2) / 2.0f;
	return cv::Vec2i((int)average[0], (int)average[1]);
}

cv::Vec2i MathTools::linesIntersection(cv::Vec2i l1p1, cv::Vec2i l1p2, cv::Vec2i l2p1, cv::Vec2i l2p2)
{
	float x;
	float y;

	x = (float)(l1p1[0] * l1p2[1] - l1p1[1] * l1p2[0]) * (l2p1[0] - l2p2[0]) - (float)(l2p1[0] * l2p2[1] - l2p1[1] * l2p2[0]) * (l1p1[0] - l1p2[0]);
	x /= (float)(l1p1[0] - l1p2[0]) * (l2p1[1] - l2p2[1]) - (float)(l1p1[1] - l1p2[1]) * (l2p1[0] - l2p2[0]);

	y = (float)(l1p1[0] * l1p2[1] - l1p1[1] * l1p2[0]) * (l2p1[1] - l2p2[1]) - (float)(l2p1[0] * l2p2[1] - l2p1[1] * l2p2[0]) * (l1p1[1] - l1p2[1]);
	y /= (float)(l1p1[0] - l1p2[0]) * (l2p1[1] - l2p2[1]) - (float)(l1p1[1] - l1p2[1]) * (l2p1[0] - l2p2[0]);

	return cv::Vec2i((int)x, (int)y);
}

float MathTools::mod2Pi(float val)
{
	if (val > M_PI) {
		return val - 2 * M_PI;
	}

	if (val < - M_PI) {
		return val + 2 * M_PI;
	}

	return val;
}

float MathTools::lineOrientation(cv::Vec2i lp1, cv::Vec2i lp2)
{
	float y = (float)lp2[0] - (float)lp1[0];
	float x = (float)lp2[1] - (float)lp1[1];

	return atan2f(-y, x);
}

float MathTools::pointLineDistance(cv::Vec2i lp1, cv::Vec2i lp2, cv::Vec2i p)
{
	float temp = (float)abs((lp2[0] - lp1[0]) * (lp1[1] - p[1]) - (lp2[1] - lp1[1]) * (lp1[0] - p[0]));
	temp /= (float)sqrt((lp2[0] - lp1[0]) * (lp2[0] - lp1[0]) + (lp2[1] - lp1[1]) * (lp2[1] - lp1[1]));
	return temp;
}

float MathTools::pointPointDistance(cv::Vec2i p1, cv::Vec2i p2)
{
	cv::Vec2i delta = p2 - p1;
	return (float)sqrt(delta[0] * delta[0] + delta[1] * delta[1]);
}
