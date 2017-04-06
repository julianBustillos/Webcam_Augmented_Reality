#pragma once

#include <opencv2/opencv.hpp>
#include "cornerDetector.h"
#include <chrono>


enum class FPS {
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
	SUPERPOSITION,
	SIZE
};


class DebugInfo {
public:
	DebugInfo();
	~DebugInfo();
	void nextFPS();
	void nextMode();
	void printOnFrame(cv::Mat & frame, const CornerDetector & detector);
	void parametersWindow();

private:
	void updateFPS(const CornerDetector & detector);
	void print(cv::Mat & frame, const CornerDetector & detector) const;
	void printFPS(cv::Mat & frame, const CornerDetector & detector) const;
	void printRegions(cv::Mat & frame, const CornerDetector & detector) const;
	void printEdgels(cv::Mat & frame, const CornerDetector & detector) const;
	void printEdgel(cv::Mat & frame, const cv::Vec2i position, const cv::Scalar color) const;
	void printLines(cv::Mat & frame, const CornerDetector & detector) const;
	void printLineList(cv::Mat & frame, const std::vector<Line> & lineList, const cv::Scalar color) const;
	void printMergedLines(cv::Mat & frame, const CornerDetector & detector) const;

	// DATA
	FPS fps;
	Mode mode;
	std::chrono::steady_clock::time_point start;
	std::chrono::steady_clock::time_point end;
	int fpsCounter;
	int realFps;
	int theoricalFps;
	double execTime;
	const std::string windowName;
};

void callbackInt(int val, void *data);
void callbackIntNotNull(int val, void *data);
void callbackFloatDegree(int val, void *data);
void callbackFloatDist(int val, void *data);
