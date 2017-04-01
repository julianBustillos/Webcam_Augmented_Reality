#pragma once

#include <opencv2/opencv.hpp>
#include <time.h>


enum class FPS {
	DISABLED,
	ENABLED,
	SIZE
};

enum class Mode {
	CLASSIC,
	POINTS,
	SIZE
};


class DebugInfo {
public:
	DebugInfo();
	~DebugInfo();
	void nextFPS();
	void printOnFrame(cv::Mat & frame);

private:
	FPS fps;
	Mode mode;
	time_t start;
	time_t end;
	int fpsCounter;
	int actualFps;
	void updateFPS();
	void print(cv::Mat & frame);

};
