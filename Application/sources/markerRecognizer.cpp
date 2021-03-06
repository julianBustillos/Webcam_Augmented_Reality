#include "markerRecognizer.h"
#include "constants.h"
#include "mathTools.h"


MarkerRecognizer::MarkerRecognizer(int width, int height) :
	found(false), foundOnce(false), lastFoundFrame(GET(MAX_FRAMES))
{
	frameSize[0] = width;
	frameSize[1] = height;

	orderedCorners.resize(4, cv::Vec2i(0, 0));
	worldCorners.resize(4);

	worldCorners[0] = cv::Vec2d(0., 0.);
	worldCorners[1] = cv::Vec2d(0., 10.);
	worldCorners[2] = cv::Vec2d(10., 10.);
	worldCorners[3] = cv::Vec2d(10., 0.);

	computeDirectionIndices();
}

MarkerRecognizer::~MarkerRecognizer()
{
}

void MarkerRecognizer::searchMarker(const cv::Mat & frame, const std::vector<std::vector<cv::Vec2i>> & cornerGroups) {
	found = false;
	lastFoundFrame++;
	Direction dir;

	for (int idx = 0; idx < (int)cornerGroups.size(); idx++) {
		MathTools::findHomography(cornerGroups[idx], worldCorners, homography);
		dir = getDirection(frame);
		if (dir != Direction::UNKNOWN) {
#ifdef DEBUG
			currentDir = dir;
			computeDirectionTriangle();
#endif
			unorderedCorners = cornerGroups[idx];
			computeOrderedCorners(cornerGroups[idx], dir);
			found = true;
			foundOnce = true;
			lastFoundFrame = 0;
			return;
		}
	}

	if (foundOnce) {
		MathTools::findHomography(unorderedCorners, worldCorners, homography);
		dir = getDirection(frame);
		if (dir != Direction::UNKNOWN) {
			computeOrderedCorners(unorderedCorners, dir);
			found = true;
			lastFoundFrame = 0;
		}
	}
}

bool MarkerRecognizer::identified() const {
	return found || (lastFoundFrame < GET(MAX_FRAMES));
}

std::vector<cv::Vec2i> MarkerRecognizer::getOrderedCorners() const {
	return orderedCorners;
}

std::vector<cv::Vec2i> MarkerRecognizer::getDirectionTriangle() const {
	return triangle;
}

std::vector<cv::Vec2i> MarkerRecognizer::getROI() const
{
	std::vector<cv::Vec2i> ROI;
	ROI.clear();

	if (identified()) {
		cv::Vec2i min = orderedCorners[0];
		cv::Vec2i max = orderedCorners[0];

		for (int idx = 0; idx < 4; idx++) {
			min[0] = std::min(min[0], orderedCorners[idx][0]);
			min[1] = std::min(min[1], orderedCorners[idx][1]);
			max[0] = std::max(max[0], orderedCorners[idx][0]);
			max[1] = std::max(max[1], orderedCorners[idx][1]);
		}

		min = min - GET(ROI_MARGIN) / 100.0f * (max - min);
		max = max + GET(ROI_MARGIN) / 100.0f * (max - min);

		min[0] = std::max(0, min[0]);
		min[1] = std::max(0, min[1]);
		max[0] = std::min(frameSize[1], max[0]);
		max[1] = std::min(frameSize[0], max[1]);

		ROI.push_back(min);
		ROI.push_back(max);
	}

	return ROI;
}

cv::Mat MarkerRecognizer::getMarkerMatrix() {
	cv::Mat marker = cv::Mat::zeros(6, 6, CV_16S);
	int index = GET(ARTAG_ID);
	blackElementCount = 0;

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			marker.at<short>(j, i) = (index >> (i + j * 6)) & 1;
			if (marker.at<short>(j, i) == 1) {
				blackElementCount++;  
			}
		}
	}

	return marker;
}

int MarkerRecognizer::getRotateIdentifier(const cv::Mat & marker, int initI, int incrI, int initJ, int incrJ, bool reverse) const
{
	int identifier = 0;
	int shiftI, shiftJ = 0;
	for (int j = initJ; std::min(j, initJ + 6 * incrJ) < std::max(j, initJ + 6 * incrJ); j += incrJ) {
		shiftI = 0;
		for (int i = initI; std::min(i, initI + 6 * incrI) < std::max(i, initI + 6 * incrI); i += incrI) {
			if (reverse) {
				identifier += marker.at<short>(j, i) << (shiftI + shiftJ * 6);
			}
			else {
				identifier += marker.at<short>(i, j) << (shiftI + shiftJ * 6);
			}
			shiftI++;
		}
		shiftJ++;
	}

	return identifier;
}

void MarkerRecognizer::computeDirectionIndices() {
	cv::Mat marker = getMarkerMatrix();

	directionIndices[(int)Direction::UP] = getRotateIdentifier(marker, 0, 1, 0, 1, true);
	directionIndices[(int)Direction::RIGHT] = getRotateIdentifier(marker, 5, -1, 0, 1, false);
	directionIndices[(int)Direction::DOWN] = getRotateIdentifier(marker, 5, -1, 5, -1, true);
	directionIndices[(int)Direction::LEFT] = getRotateIdentifier(marker, 0, 1, 5, -1, false);
}

cv::Vec2i MarkerRecognizer::getFrameCoordinates(double worldX, double worldY) const
{
	cv::Mat point = homography * cv::Mat(cv::Vec3d(worldX, worldY, 1.0f));
	point /= point.at<double>(2, 0);

	for (int dim = 0; dim < 2; dim++) {
		if (point.at<double>(dim, 0) < 0.0f) {
			point.at<double>(dim, 0) = 0.0f;
		}
		else if (point.at<double>(dim, 0) >= frameSize[1 - dim]) {
			point.at<double>(dim, 0) = (double)frameSize[1 - dim] - 1.0f;
		}
	}

	return cv::Vec2i((int)point.at<double>(0, 0), (int)point.at<double>(1, 0));
}

int MarkerRecognizer::getColor(const cv::Mat & frame, int i, int j) const
{
	double grayVal = 0.0f;
	for (double y = j + 2.2f; y < j + 3.0f; y += 0.2f) {
		for (double x = i + 2.2f; x < i + 3.0f; x += 0.2f) {
			grayVal += MathTools::grayScaleValue(frame, getFrameCoordinates(x, y));
		}
	}

	return (int)(grayVal / 16);
}

Direction MarkerRecognizer::getDirection(const cv::Mat & frame) const {
	cv::Mat matrix = cv::Mat::zeros(6, 6, CV_16S);
	std::vector<int> grayValues;
	grayValues.clear();
	int identifier;

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			matrix.at<short>(i, j) = getColor(frame, i, j);
			grayValues.push_back(matrix.at<short>(i, j));
		}
	}

	std::sort(grayValues.begin(), grayValues.end());

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			matrix.at<short>(i, j) = (matrix.at<short>(i, j) < grayValues[blackElementCount]) ? 1 : 0;
		}
	}

	identifier = getRotateIdentifier(matrix, 0, 1, 0, 1, true);
	
	if (identifier == directionIndices[(int)Direction::UP]) {
		return Direction::UP;
	}

	if (identifier == directionIndices[(int)Direction::RIGHT]) {
		return Direction::RIGHT;
	}

	if (identifier == directionIndices[(int)Direction::DOWN]) {
		return Direction::DOWN;
	}

	if (identifier == directionIndices[(int)Direction::LEFT]) {
		return Direction::LEFT;
	}

	return Direction::UNKNOWN;
}

void MarkerRecognizer::computeOrderedCorners(const std::vector<cv::Vec2i> corners, Direction dir) {

	switch (dir) {
	case Direction::UP:
		orderedCorners[0] = corners[0];
		orderedCorners[1] = corners[1];
		orderedCorners[2] = corners[2];
		orderedCorners[3] = corners[3];
		break;
	case Direction::RIGHT:
		orderedCorners[0] = corners[1];
		orderedCorners[1] = corners[2];
		orderedCorners[2] = corners[3];
		orderedCorners[3] = corners[0];
		break;
	case Direction::DOWN:
		orderedCorners[0] = corners[2];
		orderedCorners[1] = corners[3];
		orderedCorners[2] = corners[0];
		orderedCorners[3] = corners[1];
		break;
	case Direction::LEFT:
		orderedCorners[0] = corners[3];
		orderedCorners[1] = corners[0];
		orderedCorners[2] = corners[1];
		orderedCorners[3] = corners[2];
		break;
	default:
		break;
	}
}

void MarkerRecognizer::computeDirectionTriangle()
{
	triangle.clear();

	switch (currentDir) {
	case Direction::UP:
		triangle.push_back(getFrameCoordinates(3.0f, 5.0f));
		triangle.push_back(getFrameCoordinates(7.0f, 7.0f));
		triangle.push_back(getFrameCoordinates(7.0f, 3.0f));
		break;
	case Direction::LEFT:
		triangle.push_back(getFrameCoordinates(5.0f, 3.0f));
		triangle.push_back(getFrameCoordinates(7.0f, 7.0f));
		triangle.push_back(getFrameCoordinates(3.0f, 7.0f));
		break;
	case Direction::DOWN:
		triangle.push_back(getFrameCoordinates(7.0f, 5.0f));
		triangle.push_back(getFrameCoordinates(3.0f, 3.0f));
		triangle.push_back(getFrameCoordinates(3.0f, 7.0f));
		break;
	case Direction::RIGHT:
		triangle.push_back(getFrameCoordinates(5.0f, 7.0f));
		triangle.push_back(getFrameCoordinates(3.0f, 3.0f));
		triangle.push_back(getFrameCoordinates(7.0f, 3.0f));
		break;
	default:
		break;
	}
}
