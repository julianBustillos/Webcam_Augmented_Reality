#include "debugInfo.h"
#include <iostream>


DebugInfo::DebugInfo() :
	fps(FPS::DISABLED), mode(Mode::CLASSIC), start(time(NULL)), end(time(NULL)), fpsCounter(0), actualFps(0)
{
	std::cout << "DEBUG MODE ENABLED" << std::endl << std::endl;
}

DebugInfo::~DebugInfo()
{
}

void DebugInfo::nextFPS()
{
	fps = FPS(((int)fps + 1) % (int)FPS::SIZE);
}

void DebugInfo::printOnFrame(cv::Mat & frame)
{
	updateFPS();
	print(frame);
}

void DebugInfo::updateFPS()
{
	fpsCounter++;
	end = time(NULL);
	if (difftime(end, start) >= 1.0) {
		actualFps = fpsCounter;
		fpsCounter = 0;
		start = end;
	}
}

void DebugInfo::print(cv::Mat & frame)
{
	if (fps == FPS::ENABLED) {
		cv::putText(frame, "FPS : " + std::to_string(actualFps), cvPoint(3, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 187, 0));
	}
}
