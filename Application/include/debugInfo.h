#pragma once

#include <opencv2/opencv.hpp>
#include "cornerDetector.h"
#include <chrono>
#include "markerRecognizer.h"


enum class Active {
	DISABLED,
	ENABLED,
	SIZE
};

enum class Mode {
	NORMAL,
	REGIONS,
	EDGELS,
	LINES,
	MERGED,
	EXTENDED,
	SUPERPOSITION,
	CORNERS,
	MARKER,
	SIZE
};


class DebugInfo {
public:
	DebugInfo();
	~DebugInfo();
	void nextFPS();
	void nextMode();
	void nextPause();
	void printOnFrame(cv::Mat & frame, double time, const CornerDetector & detector, const MarkerRecognizer & recognizer);
	void parametersWindow();
	bool isPaused() const;
	bool showMesh() const;

private:
	void updateFPS(double time);
	void print(cv::Mat & frame, const CornerDetector & detector, const MarkerRecognizer & recognizer) const;
	void printFPS(cv::Mat & frame) const;
	void printRegions(cv::Mat & frame, const CornerDetector & detector) const;
	void printEdgels(cv::Mat & frame, const CornerDetector & detector) const;
	void printPoint(cv::Mat & frame, const cv::Vec2i position, int size, const cv::Scalar color) const;
	void printLines(cv::Mat & frame, const CornerDetector & detector) const;
	void printLineList(cv::Mat & frame, const std::vector<Line> & lineList, const cv::Scalar color) const;
	void printMergedLines(cv::Mat & frame, const CornerDetector & detector) const;
	void printExtendedLines(cv::Mat & frame, const CornerDetector & detector) const;
	void printCorners(cv::Mat & frame, const CornerDetector & detector) const;
	void printTriangle(cv::Mat & frame, const std::vector<cv::Vec2i> & pointList, cv::Scalar color) const;
	void printMarker(cv::Mat & frame, const MarkerRecognizer & recognizer) const;

	// DATA
	Active fps;
	Mode mode;
	Active pause;
	std::chrono::steady_clock::time_point start;
	std::chrono::steady_clock::time_point end;
	int fpsCounter;
	int realFps;
	int theoricalFps;
	double execTime;
	const std::string windowName;
};

// Callback functions for parameter window
void callbackInt(int val, void *data);
void callbackIntNotNull(int val, void *data);
void callbackFloatDegree(int val, void *data);
void callbackFloatDist(int val, void *data);
