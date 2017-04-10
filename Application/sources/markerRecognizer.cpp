#include "markerRecognizer.h"
#include "constants.h"


MarkerRecognizer::MarkerRecognizer() :
	found(false)
{
	A = cv::Mat::zeros(8, 9, CV_32F);
	orderedCorners.resize(4, cv::Vec2i(0, 0));
	worldCorners.resize(4);

	worldCorners[0] = cv::Vec2i(0, 0);
	worldCorners[1] = cv::Vec2i(0, 1);
	worldCorners[2] = cv::Vec2i(1, 1);
	worldCorners[3] = cv::Vec2i(1, 0);

	computeDirectionIndices();

}

MarkerRecognizer::~MarkerRecognizer()
{
}

void MarkerRecognizer::searchMarker(const cv::Mat & frame, const std::vector<std::vector<cv::Vec2i>> & cornerGroups) {
	found = false;
	Direction dir;

	for (int idx = 0; idx < cornerGroups.size(); idx++) {
		setA(cornerGroups[idx]);
		solveH();
		dir = getDirection(frame);
		if (dir != Direction::UNKNOWN) {
			computeOrderedCorners(cornerGroups[idx], dir);
			found = true;
			break;
		}
	}

}

bool MarkerRecognizer::identified() const {
	return found;
}

std::vector<cv::Vec2i> MarkerRecognizer::getOrderedCorners() const {
	return orderedCorners;
}

std::vector<cv::Vec2i> MarkerRecognizer::getDirectionTriangle() const {
	std::vector<cv::Vec2i> triangle;
	triangle.clear();

	switch (currentDir) {
	case Direction::UP:
		triangle.push_back(getFrameCoordinates(0.5f, 0.3f));
		triangle.push_back(getFrameCoordinates(0.7f, 0.7f));
		triangle.push_back(getFrameCoordinates(0.3f, 0.7f));
		break;
	case Direction::LEFT:
		triangle.push_back(getFrameCoordinates(0.3f, 0.5f));
		triangle.push_back(getFrameCoordinates(0.7f, 0.7f));
		triangle.push_back(getFrameCoordinates(0.7f, 0.3f));
		break;
	case Direction::DOWN:
		triangle.push_back(getFrameCoordinates(0.5f, 0.7f));
		triangle.push_back(getFrameCoordinates(0.3f, 0.3f));
		triangle.push_back(getFrameCoordinates(0.7f, 0.3f));
		break;
	case Direction::RIGHT:
		triangle.push_back(getFrameCoordinates(0.7f, 0.5f));
		triangle.push_back(getFrameCoordinates(0.3f, 0.3f));
		triangle.push_back(getFrameCoordinates(0.3f, 0.7f));
		break;
	default:
		break;
	}
	return triangle;
}

cv::Mat MarkerRecognizer::getMarkerMatrix() const {
	cv::Mat marker = cv::Mat::zeros(6, 6, CV_16S);
	int index = GET(ARTAG_ID);

	for (int j = 0; j < 6; j++) {
		for (int i = 0; i < 6; i++) {
			marker.at<short>(j, i) = (index >> (i + j * 6)) & 1;
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
	directionIndices[(int)Direction::RIGHT] = getRotateIdentifier(marker, 0, 1, 5, -1, false);
	directionIndices[(int)Direction::DOWN] = getRotateIdentifier(marker, 5, -1, 5, -1, true);
	directionIndices[(int)Direction::LEFT] = getRotateIdentifier(marker, 5, -1, 0, 1, false);
}

void MarkerRecognizer::setA(const std::vector<cv::Vec2i> & corners) {
	for (int pIdx = 0; pIdx < 4; pIdx++) {
		A.at<float>(2 * pIdx, 3) = (float)-worldCorners[pIdx][0];
		A.at<float>(2 * pIdx, 4) = (float)-worldCorners[pIdx][1];
		A.at<float>(2 * pIdx, 5) = -1.0f;

		A.at<float>(2 * pIdx, 6) = (float)corners[pIdx][1] * worldCorners[pIdx][0];
		A.at<float>(2 * pIdx, 7) = (float)corners[pIdx][1] * worldCorners[pIdx][1];
		A.at<float>(2 * pIdx, 8) = (float)corners[pIdx][1];

		A.at<float>(2 * pIdx + 1, 0) = (float)worldCorners[pIdx][0];
		A.at<float>(2 * pIdx + 1, 1) = (float)worldCorners[pIdx][1];
		A.at<float>(2 * pIdx + 1, 2) = 1.0f;

		A.at<float>(2 * pIdx + 1, 6) = (float)-corners[pIdx][0] * worldCorners[pIdx][0];
		A.at<float>(2 * pIdx + 1, 7) = (float)-corners[pIdx][0] * worldCorners[pIdx][1];
		A.at<float>(2 * pIdx + 1, 8) = (float)-corners[pIdx][0];
	}
}

void MarkerRecognizer::solveH() {
	cv::SVD::solveZ(A, h);
	h = h.reshape(1, 3);
}

cv::Vec2i MarkerRecognizer::getFrameCoordinates(float worldX, float worldY) const
{
	cv::Mat point = h * cv::Mat(cv::Vec3f(worldX, worldY, 1.0f));
	point /= point.at<float>(2, 0);
	
	return cv::Vec2i((int)point.at<float>(0, 0), (int)point.at<float>(1, 0));
}

Direction MarkerRecognizer::getDirection(const cv::Mat & frame) const {
	//TODO
	return Direction::RIGHT;
}

void MarkerRecognizer::computeOrderedCorners(const std::vector<cv::Vec2i> corners, Direction dir) {

#ifdef DEBUG
	currentDir = dir;
#endif
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
