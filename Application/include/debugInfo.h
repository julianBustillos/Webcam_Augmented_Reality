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
	void printOnFrame(cv::Mat & frame, FrameProcessing & processing);

private:
	void updateFPS();
	void print(cv::Mat & frame, FrameProcessing & processing);
	void printRegions(cv::Mat & frame, FrameProcessing & processing);
	void printEdgels(cv::Mat & frame, FrameProcessing & processing);
	void printEdgel(cv::Mat & frame, cv::Vec2i position, cv::Scalar color);
	void printLines(cv::Mat & frame, FrameProcessing & processing);
	void printLineList(cv::Mat & frame, std::vector<Line> & lineList, cv::Scalar color);
	void printMergedLines(cv::Mat & frame, FrameProcessing & processing);

	// DATA
	FPS fps;
	Mode mode;
	time_t start;
	time_t end;
	int fpsCounter;
	int actualFps;
};
