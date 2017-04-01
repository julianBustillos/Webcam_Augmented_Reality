#include "mathTools.h"

int MathTools::convolution(cv::Mat & frame, cv::Vec2i frameSize, std::vector<int> filter, cv::Vec2i position, cv::Vec2i scanDir, int channel)
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
