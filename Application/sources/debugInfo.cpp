#include "debugInfo.h"
#include <iostream>
#include "constants.h"
#include <opencv/highgui.h>


DebugInfo::DebugInfo() :
	fps(FPS::DISABLED), mode(Mode::NORMAL), start(time(NULL)), end(time(NULL)), fpsCounter(0), actualFps(0),
	isWindowOpen(false), windowName("DEBUG - PARAMETERS")
{
	std::cout << "### DEBUG MODE ENABLED" << std::endl;
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
	case Mode::REGIONS:
		std::cout << "REGIONS MODE" << std::endl;
		break;
	case Mode::EDGELS:
		std::cout << "EDGELS MODE" << std::endl;
		break;
	case Mode::LINES:
		std::cout << "LINES MODE" << std::endl;
		break;
	case Mode::MERGED:
		std::cout << "MERGED MODE" << std::endl;
		break;
	case Mode::SUPERPOSITION:
		std::cout << "SUPERPOSITION MODE" << std::endl;
		break;
	default:
		break;
	}
}

void DebugInfo::printOnFrame(cv::Mat & frame, const FrameProcessing & processing)
{
	updateFPS();
	print(frame, processing);
}

void DebugInfo::parametersWindow()
{
	if (isWindowOpen) {
		closeParametersWindow();
	}
	else {
		openParametersWindow();
	}

	isWindowOpen = !isWindowOpen;
}

void test(int val, void *data) {
	std::cout << "CALLBACK" << std::endl;
}

void DebugInfo::openParametersWindow() const
{
	// Create window
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

	//Create trackbars
	cv::createTrackbar("test", windowName, &CONSTANTS::REGION_PIXEL_SIZE, 100, test);
}

void DebugInfo::closeParametersWindow() const
{
	cv::destroyWindow(windowName);
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

void DebugInfo::print(cv::Mat & frame, const FrameProcessing & processing) const
{
	if (fps == FPS::ENABLED) {
		cv::putText(frame, "FPS : " + std::to_string(actualFps), cvPoint(3, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 187, 0));
	}

	switch (mode) {
	case Mode::REGIONS:
		printRegions(frame, processing);
		break;
	case Mode::EDGELS:
		printEdgels(frame, processing);
		break;
	case Mode::LINES:
		printLines(frame, processing);
		break;
	case Mode::MERGED:
		printMergedLines(frame, processing);
		break;
	case Mode::SUPERPOSITION:
		printRegions(frame, processing);
		printEdgels(frame, processing);
		printMergedLines(frame, processing);
		printLines(frame, processing);
		break;
	default:
		break;
	}
}

void DebugInfo::printRegions(cv::Mat & frame, const FrameProcessing & processing) const
{
	cv::Scalar orange(255, 128, 0);

	int iMin = std::max(0, processing.getRegionOrigin()[0]);
	int iMax = std::min(frame.size().height, iMin + processing.getRegionNumber()[0] * CONSTANTS::REGION_PIXEL_SIZE);

	int jMin = std::max(0, processing.getRegionOrigin()[1]);
	int jMax = std::min(frame.size().width, jMin + processing.getRegionNumber()[1] * CONSTANTS::REGION_PIXEL_SIZE);

	for (int i = iMin; i < iMax; i += CONSTANTS::REGION_PIXEL_SIZE) {
		cv::line(frame, cv::Vec2i(jMin, i), cv::Vec2i(jMax, i), orange, 1);
	}

	for (int j = jMin; j < jMax; j += CONSTANTS::REGION_PIXEL_SIZE) {
		cv::line(frame, cv::Vec2i(j, iMin), cv::Vec2i(j, iMax), orange, 1);
	}
}

void DebugInfo::printEdgels(cv::Mat & frame, const FrameProcessing & processing) const
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

void DebugInfo::printEdgel(cv::Mat & frame, const cv::Vec2i position, const cv::Scalar color) const
{
	cv::Vec2i reversePos(position[1], position[0]);
	cv::Point rect[4] = { reversePos + cv::Vec2i(-1, -1), reversePos + cv::Vec2i(-1, 1), reversePos + cv::Vec2i(1, 1), reversePos + cv::Vec2i(1, -1) };
	const cv::Point *cRectPtr[4] = { rect };
	int nbPts = 4;
	cv::fillPoly(frame, cRectPtr, &nbPts, 1, color);
}

void DebugInfo::printLines(cv::Mat & frame, const FrameProcessing & processing) const
{
	std::vector<Line> lineList = processing.getLineList();
	cv::Scalar red(0, 0, 255);

	printLineList(frame, lineList, red);
}

void DebugInfo::printLineList(cv::Mat & frame, const std::vector<Line>& lineList, const cv::Scalar color) const
{
	for (int k = 0; k < lineList.size(); k++) {
		cv::Vec2i reverseP1(lineList[k].p1[1], lineList[k].p1[0]);
		cv::Vec2i reverseP2(lineList[k].p2[1], lineList[k].p2[0]);
		cv::line(frame, reverseP1, reverseP2, color, 2);
	}
}

void DebugInfo::printMergedLines(cv::Mat & frame, const FrameProcessing & processing) const
{
	std::vector<Line> lineList = processing.getMergedLineList();
	cv::Scalar orange(0, 128, 255);

	printLineList(frame, lineList, orange);
}
