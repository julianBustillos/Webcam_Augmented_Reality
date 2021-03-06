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
	std::vector<cv::Vec2i> getROI() const;

private:
	cv::Mat getMarkerMatrix();
	int getRotateIdentifier(const cv::Mat & marker, int initI, int incrI, int initJ, int incrJ, bool reverse) const;
	void computeDirectionIndices();
	cv::Vec2i getFrameCoordinates(double worldX, double worldY) const;
	int getColor(const cv::Mat & frame, int i, int j) const;
	Direction getDirection(const cv::Mat & frame) const;
	void computeOrderedCorners(const std::vector<cv::Vec2i> corners, Direction dir);
	void computeDirectionTriangle();

	// DATA
	int blackElementCount;
	int frameSize[2];
	int directionIndices[4];
	cv::Mat homography;
	bool found;
	bool foundOnce;
	int lastFoundFrame;
	std::vector<cv::Vec2i> triangle;
	std::vector<cv::Vec2d> worldCorners;
	std::vector<cv::Vec2i> unorderedCorners;
	std::vector<cv::Vec2i> orderedCorners;
	Direction currentDir;
};
