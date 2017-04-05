#pragma once

#include <opencv2/opencv.hpp>
#include <time.h>
#include "frameProcessing.h"


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
	void printOnFrame(cv::Mat & frame, const FrameProcessing & processing);
	void parametersWindow();

private:
	void updateFPS();
	void print(cv::Mat & frame, const FrameProcessing & processing) const;
	void printRegions(cv::Mat & frame, const FrameProcessing & processing) const;
	void printEdgels(cv::Mat & frame, const FrameProcessing & processing) const;
	void printEdgel(cv::Mat & frame, const cv::Vec2i position, const cv::Scalar color) const;
	void printLines(cv::Mat & frame, const FrameProcessing & processing) const;
	void printLineList(cv::Mat & frame, const std::vector<Line> & lineList, const cv::Scalar color) const;
	void printMergedLines(cv::Mat & frame, const FrameProcessing & processing) const;

	// DATA
	FPS fps;
	Mode mode;
	time_t start;
	time_t end;
	int fpsCounter;
	int actualFps;
	const std::string windowName;
};

void callbackInt(int val, void *data);
void callbackIntNotNull(int val, void *data);
void callbackFloatDegree(int val, void *data);
void callbackFloatDist(int val, void *data);
