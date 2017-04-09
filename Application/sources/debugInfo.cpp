#include "debugInfo.h"
#include <iostream>
#include "constants.h"
#include <opencv/highgui.h>


DebugInfo::DebugInfo() :
	fps(Active::DISABLED), mode(Mode::NORMAL), pause(Active::DISABLED),
	start(std::chrono::steady_clock::now()), end(std::chrono::steady_clock::now()),
	fpsCounter(0), realFps(0), theoricalFps(0), execTime(0.0),
	windowName("DEBUG - PARAMETERS")
{
	std::cout << "### DEBUG MODE ENABLED ###" << std::endl << std::endl;
}

DebugInfo::~DebugInfo()
{
}

void DebugInfo::nextFPS()
{
	fps = Active(((int)fps + 1) % (int)Active::SIZE);
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
	case Mode::EXTENDED:
		std::cout << "EXTENDED MODE" << std::endl;
		break;
	case Mode::CORNERS:
		std::cout << "CORNERS MODE" << std::endl;
		break;
	case Mode::SUPERPOSITION:
		std::cout << "SUPERPOSITION MODE" << std::endl;
		break;
	default:
		break;
	}
}

void DebugInfo::nextPause()
{
	pause = Active(((int)pause + 1) % (int)Active::SIZE);
}

void DebugInfo::printOnFrame(cv::Mat & frame, const CornerDetector & detector)
{
	updateFPS(detector);
	print(frame, detector);
}

void DebugInfo::parametersWindow()
{
	// Create window
	cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO | CV_GUI_NORMAL);
	cv::resizeWindow(windowName, 290, 470);

	//Create trackbars
	cv::createTrackbar("INTENSITY", windowName, nullptr, 200, callbackInt, PTR(INTENSITY_THRESHOLD));
	cv::createTrackbar("CHANNEL", windowName, nullptr, 256, callbackInt, PTR(CHANNEL_GAP_THRESHOLD));
	cv::createTrackbar("ORIENT", windowName, nullptr, 360, callbackFloatDegree, PTR(ORIENTATION_TOLERANCE));
	cv::createTrackbar("HLINE", windowName, nullptr, 200, callbackInt, PTR(HYPOLINE_ATTEMPTS));
	cv::createTrackbar("PL_DIST", windowName, nullptr, 300, callbackFloatDist, PTR(POINT_LINE_DIST_TOLERANCE));
	cv::createTrackbar("DL_SEARCH", windowName, nullptr, 200, callbackInt, PTR(DOMINANT_LINE_SEARCH_ATTEMPTS));
	cv::createTrackbar("DL_VOTES", windowName, nullptr, 30, callbackInt, PTR(MIN_DOMINANT_LINE_VOTES));
	cv::createTrackbar("L_SEARCH", windowName, nullptr, 100, callbackInt, PTR(MAX_LINE_SEARCH_ITER));
	cv::createTrackbar("BRIGHT", windowName, nullptr, 255, callbackInt, PTR(MIN_BRIGHTNESS));
	cv::createTrackbar("CORNERS", windowName, nullptr, 1000, callbackFloatDist, PTR(MAX_DIST_CORNERS));
	cv::createTrackbar("PARALLEL", windowName, nullptr, 360, callbackFloatDegree, PTR(PARALLELISM_TOLERANCE));

	// Set trackbars values
	cv::setTrackbarPos("INTENSITY", windowName, GET(INTENSITY_THRESHOLD));
	cv::setTrackbarPos("CHANNEL", windowName, GET(CHANNEL_GAP_THRESHOLD));
	cv::setTrackbarPos("ORIENT", windowName, (int)(GET(ORIENTATION_TOLERANCE) * 180 / M_PI));
	cv::setTrackbarPos("HLINE", windowName, GET(HYPOLINE_ATTEMPTS));
	cv::setTrackbarPos("PL_DIST", windowName, (int)(GET(POINT_LINE_DIST_TOLERANCE) * 100));
	cv::setTrackbarPos("DL_SEARCH", windowName, GET(DOMINANT_LINE_SEARCH_ATTEMPTS));
	cv::setTrackbarPos("DL_VOTES", windowName, GET(MIN_DOMINANT_LINE_VOTES));
	cv::setTrackbarPos("L_SEARCH", windowName, GET(MAX_LINE_SEARCH_ITER));
	cv::setTrackbarPos("BRIGHT", windowName, GET(MIN_BRIGHTNESS));
	cv::setTrackbarPos("CORNERS", windowName, (int)(GET(MAX_DIST_CORNERS) * 100));
	cv::setTrackbarPos("PARALLEL", windowName, (int)(GET(PARALLELISM_TOLERANCE) * 180 / M_PI));
}

bool DebugInfo::isPaused() const
{
	return pause == Active::ENABLED;
}

void DebugInfo::updateFPS(const CornerDetector & detector)
{
	end = std::chrono::steady_clock::now();
	fpsCounter++;
	execTime += detector.getLastExecTime();
	if (std::chrono::duration<double>(end - start).count() >= 1.0) {
		realFps = (int)ceil(fpsCounter / std::chrono::duration<double>(end - start).count());
		theoricalFps = (int)ceil(fpsCounter / execTime);
		fpsCounter = 0;
		start = end;
		execTime = 0.0;
	}
}

void DebugInfo::print(cv::Mat & frame, const CornerDetector & detector) const
{
	switch (mode) {
	case Mode::REGIONS:
		printRegions(frame, detector);
		break;
	case Mode::EDGELS:
		printEdgels(frame, detector);
		break;
	case Mode::LINES:
		printLines(frame, detector);
		break;
	case Mode::MERGED:
		printMergedLines(frame, detector);
		break;
	case Mode::EXTENDED:
		printExtendedLines(frame, detector);
		break;
	case Mode::CORNERS:
		printCorners(frame, detector);
		break;
	case Mode::SUPERPOSITION:
		printRegions(frame, detector);
		printEdgels(frame, detector);
		printExtendedLines(frame, detector);
		printMergedLines(frame, detector);
		printLines(frame, detector);
		break;
	default:
		break;
	}

	if (fps == Active::ENABLED) {
		printFPS(frame, detector);
	}
}

void DebugInfo::printFPS(cv::Mat & frame, const CornerDetector & detector) const
{
	cv::putText(frame, "FPS (real)      : " + std::to_string(realFps), cvPoint(3, 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(102, 0, 0));
	cv::putText(frame, "FPS (theorical) : " + std::to_string(theoricalFps), cvPoint(3, 45), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(204, 0, 102));
}

void DebugInfo::printRegions(cv::Mat & frame, const CornerDetector & detector) const
{
	cv::Scalar orange(255, 128, 0);

	int iMin = std::max(0, detector.getRegionOrigin()[0]);
	int iMax = std::min(frame.size().height, iMin + detector.getRegionNumber()[0] * GET(REGION_PIXEL_SIZE));

	int jMin = std::max(0, detector.getRegionOrigin()[1]);
	int jMax = std::min(frame.size().width, jMin + detector.getRegionNumber()[1] * GET(REGION_PIXEL_SIZE));

	for (int i = iMin; i < iMax; i += GET(REGION_PIXEL_SIZE)) {
		cv::line(frame, cv::Vec2i(jMin, i), cv::Vec2i(jMax, i), orange, 1);
	}

	for (int j = jMin; j < jMax; j += GET(REGION_PIXEL_SIZE)) {
		cv::line(frame, cv::Vec2i(j, iMin), cv::Vec2i(j, iMax), orange, 1);
	}
}

void DebugInfo::printEdgels(cv::Mat & frame, const CornerDetector & detector) const
{
	std::vector<Edgel> edgelList = detector.getEdgelList();
	cv::Scalar blue(255, 0, 0);
	cv::Scalar green(0, 255, 0);

	for (int k = 0; k < edgelList.size(); k++) {
		if (edgelList[k].type == EdgelType::horizontal) {
			printPoint(frame, edgelList[k].position, 3, green);
		}
		else {
			printPoint(frame, edgelList[k].position, 3, blue);
		}
	}
}

void DebugInfo::printPoint(cv::Mat & frame, const cv::Vec2i position, int size, const cv::Scalar color) const
{
	cv::Vec2i reversePos(position[1], position[0]);
	int offset = (size - 1) / 2;
	cv::Point rect[4] = { reversePos + cv::Vec2i(-offset, -offset), reversePos + cv::Vec2i(-offset, offset), reversePos + cv::Vec2i(offset, offset), reversePos + cv::Vec2i(offset, -offset) };
	const cv::Point *cRectPtr[4] = { rect };
	int nbPts = 4;
	cv::fillPoly(frame, cRectPtr, &nbPts, 1, color);
}

void DebugInfo::printLines(cv::Mat & frame, const CornerDetector & detector) const
{
	std::vector<Line> lineList = detector.getLineList();
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

void DebugInfo::printMergedLines(cv::Mat & frame, const CornerDetector & detector) const
{
	std::vector<Line> lineList = detector.getMergedLineList();
	cv::Scalar orange(0, 128, 255);

	printLineList(frame, lineList, orange);
}

void DebugInfo::printExtendedLines(cv::Mat & frame, const CornerDetector & detector) const
{
	std::vector<Line> lineList = detector.getExtendedLineList();
	std::vector<Line> validLineList;
	std::vector<Line> invalidLineList;
	cv::Scalar yellow(0, 255, 255);
	cv::Scalar white(255, 255, 255);
	validLineList.clear();
	invalidLineList.clear();

	for (int idx = 0; idx < lineList.size(); idx++) {
		if (lineList[idx].isValid) {
			validLineList.push_back(lineList[idx]);
		}
		else {
			invalidLineList.push_back(lineList[idx]);
		}
	}

	printLineList(frame, invalidLineList, white);
	printLineList(frame, validLineList, yellow);
}

void DebugInfo::printCorners(cv::Mat & frame, const CornerDetector & detector) const
{
	std::vector<std::vector<cv::Vec2i>> cornerGroupList = detector.getCornerGroupsList();
	Line currentLine;
	std::vector<Line> lineList;
	lineList.clear();
	cv::Scalar purple(255, 51, 153);
	cv::Scalar purpleBright(255, 102, 178);
	cv::Vec2i prec;

	//Print link
	for (int groupIdx = 0; groupIdx < cornerGroupList.size(); groupIdx++) {
		prec = cornerGroupList[groupIdx][cornerGroupList[groupIdx].size() - 1];
		for (int cornerIdx = 0; cornerIdx < cornerGroupList[groupIdx].size(); cornerIdx++) {
			currentLine.p1 = prec;
			currentLine.p2 = cornerGroupList[groupIdx][cornerIdx];
			prec = cornerGroupList[groupIdx][cornerIdx];
			lineList.push_back(currentLine);
		}
	}

	printLineList(frame, lineList, purple);

	//Print corners
	for (int groupIdx = 0; groupIdx < cornerGroupList.size(); groupIdx++) {
		for (int cornerIdx = 0; cornerIdx < cornerGroupList[groupIdx].size(); cornerIdx++) {
			printPoint(frame, cornerGroupList[groupIdx][cornerIdx], 7, purpleBright);
		}
	}
}



// Callback functions for parameter window
void callbackInt(int val, void *data) {
	int *ptr = (int *)data;
	if (ptr) {
		*ptr = val;
	}
}

void callbackIntNotNull(int val, void * data)
{
	int *ptr = (int *)data;
	if (ptr && val > 0) {
		*ptr = val;
	}
}

void callbackFloatDegree(int val, void * data)
{
	float *ptr = (float *)data;
	if (ptr) {
		*ptr = val * M_PI / 180;
	}
}

void callbackFloatDist(int val, void * data)
{
	float *ptr = (float *)data;
	if (ptr) {
		*ptr = (float)val / 100;
	}
}
