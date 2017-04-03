#include "mathTools.h"
#include "macros.h"


int MathTools::convolution(cv::Mat & frame, cv::Vec2i & frameSize, std::vector<int> & filter, cv::Vec2i & position, cv::Vec2i & scanDir, int channel)
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
