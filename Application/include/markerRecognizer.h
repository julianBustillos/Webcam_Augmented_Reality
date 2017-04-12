#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "geometry.h"
#include "debug.h"


class MarkerRecognizer {
public:
	MarkerRecognizer(int width, int height);
	~MarkerRecognizer();
	void searchMarker(const cv::Mat & frame, const std::vector<std::vector<cv::Vec2i>> & cornerGroups);
	bool identified() const;
	std::vector<cv::Vec2i> getOrderedCorners() const;
	std::vector<cv::Vec2i> getDirectionTriangle() const;

private:
	cv::Mat getMarkerMatrix() const;
	int getRotateIdentifier(const cv::Mat & marker, int initI, int incrI, int initJ, int incrJ, bool reverse) const;
	void computeDirectionIndices();
	void setA(const std::vector<cv::Vec2i> & corners);
	void solveH();
	cv::Vec2i getFrameCoordinates(float worldX, float worldY) const;
	int getColor(const cv::Mat & frame, int i, int j) const;
	Direction getDirection(const cv::Mat & frame) const;
	void computeOrderedCorners(const std::vector<cv::Vec2i> corners, Direction dir);

	// DATA
	int frameSize[2];
	int directionIndices[4];
	cv::Mat A;
	cv::Mat h;
	bool found;
	std::vector<cv::Vec2i> worldCorners;
	std::vector<cv::Vec2i> orderedCorners;
	Direction currentDir;
};
