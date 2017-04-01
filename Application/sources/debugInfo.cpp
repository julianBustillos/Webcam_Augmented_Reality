#include "debugInfo.h"
#include <iostream>


DebugInfo::DebugInfo() :
	fps(FPS::DISABLED), mode(Mode::NORMAL), start(time(NULL)), end(time(NULL)), fpsCounter(0), actualFps(0)
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

void DebugInfo::nextMode()
{
	mode = Mode(((int)mode + 1) % (int)Mode::SIZE);
	switch (mode) {
	case Mode::NORMAL:
		std::cout << "NORMAL MODE" << std::endl;
		break;
	case Mode::EDGELS:
		std::cout << "EDGELS MODE" << std::endl;
		break;
	default:
		break;
	}
}

void DebugInfo::printOnFrame(cv::Mat & frame, FrameProcessing & processing)
{
	updateFPS();
	print(frame, processing);
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

void DebugInfo::print(cv::Mat & frame, FrameProcessing & processing)
{
	if (fps == FPS::ENABLED) {
		cv::putText(frame, "FPS : " + std::to_string(actualFps), cvPoint(3, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 187, 0));
	}

	switch (mode) {
	case Mode::EDGELS:
		printEdgels(frame, processing);
		break;
	default:
		break;
	}
}

void DebugInfo::printEdgels(cv::Mat & frame, FrameProcessing & processing)
{
	std::vector<Edgel> edgelList = processing.getEdgelList();
	cv::Scalar blue(255, 0, 0);
	cv::Scalar green(0, 255, 0);

	for (int k = 0; k < edgelList.size(); k++) {
		if (edgelList[k].type == EdgelType::horizontal) {
			printEdgel(frame, edgelList[k].position, green);
		}
		else {
			printEdgel(frame, edgelList[k].position, blue);
		}
	}
}

void DebugInfo::printEdgel(cv::Mat & frame, cv::Vec2i position, cv::Scalar color)
{
	cv::Vec2i reversePos(position[1], position[0]);
	cv::Point rect[4] = { reversePos + cv::Vec2i(-1, -1), reversePos + cv::Vec2i(-1, 1), reversePos + cv::Vec2i(1, 1), reversePos + cv::Vec2i(1, -1) };
	const cv::Point *cRectPtr[4] = { rect };
	int nbPts = 4;
	cv::fillPoly(frame, cRectPtr, &nbPts, 1, color);
}
