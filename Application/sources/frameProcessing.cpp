#include "frameProcessing.h"
#include "macros.h"
#include <iostream>
#include "mathTools.h"
#include "debugInfo.h"


FrameProcessing::FrameProcessing(int width, int height)
{
	frameSize[0] = height;
	frameSize[1] = width;
	regionOrigin[0] = 0;
	regionOrigin[1] = 0;
	regionNumber[0] = (int)ceil((float)height / REGION_PIXEL_SIZE);
	regionNumber[1] = (int)ceil((float)width / REGION_PIXEL_SIZE);
	
	regionGrid.resize(regionNumber[0]);
	for (int i = 0; i < regionGrid.size(); i++) {
		regionGrid[i].resize(regionNumber[1]);
	}

	std::cout << "Region grid size : " << regionNumber[0] << "*" << regionNumber[1] << std::endl;
}

FrameProcessing::~FrameProcessing()
{
}

void FrameProcessing::execute(cv::Mat & frame)
{
	findEdgels(frame);
}

std::vector<Edgel> FrameProcessing::getEdgelList()
{
	std::vector<Edgel> edgelList;
	edgelList.clear();

	for (int i = 0; i < regionNumber[0]; i++) {
		for (int j = 0; j < regionNumber[1]; j++) {
			for (int k = 0; k < regionGrid[i][j].edgels.size(); k++) {
				edgelList.push_back(regionGrid[i][j].edgels[k]);
			}
		}
	}

	return edgelList;
}

void FrameProcessing::reinitNeededRegions()
{
	for (int i = 0; i < regionNumber[0]; i++) {
		for (int j = 0; j < regionNumber[1]; j++) {
			regionGrid[i][j].edgels.clear();
		}
	}
}

void FrameProcessing::addEdgel(cv::Vec2i position, float orientation, EdgelType type)
{
	Edgel edgel;
	edgel.position = position;
	edgel.orientation = orientation;
	edgel.type = type;

	cv::Vec2i regionIndex;
	regionIndex[0] = (int)floor((float)(position[0]-regionOrigin[0]) / REGION_PIXEL_SIZE);
	regionIndex[1] = (int)floor((float)(position[1]-regionOrigin[1]) / REGION_PIXEL_SIZE);

	regionGrid[regionIndex[0]][regionIndex[1]].edgels.push_back(edgel);
}

void FrameProcessing::findEdgels(cv::Mat & frame)
{
	reinitNeededRegions();
	cv::Vec2i verticalScanDir(1, 0);
	scanLines(frame, verticalScanDir, EdgelType::horizontal);
	cv::Vec2i horizontalScanDir(0, 1);
	scanLines(frame, horizontalScanDir, EdgelType::vertical);
}

void FrameProcessing::scanLines(cv::Mat & frame, cv::Vec2i scanDir, EdgelType type)
{
	cv::Vec2i strideDir = cv::Vec2i(1, 1) - scanDir;
	int strideIdxFirst = regionOrigin.dot(strideDir);
	int strideIdxLast = std::min(strideIdxFirst + regionNumber.dot(strideDir) * REGION_PIXEL_SIZE, frameSize.dot(strideDir));

	int scanIdxFirst = regionOrigin.dot(scanDir);
	int scanIdxLast = std::min(scanIdxFirst + regionNumber.dot(scanDir) * REGION_PIXEL_SIZE, frameSize.dot(scanDir));
	std::vector<int> scanline(scanIdxLast - scanIdxFirst);

	std::vector<int> filter = FILTER;

	for (int strideIdx = strideIdxFirst; strideIdx < strideIdxLast; strideIdx += SCANLINE_STRIDE) {
		for (int scanIdx = 0; scanIdx < scanline.size(); scanIdx++) {

			// Get all scanline values
			cv::Vec2i position = scanIdx * scanDir + strideIdx * strideDir;
			scanline[scanIdx] = MathTools::convolution(frame, frameSize, filter, position, scanDir, 0);

		}
			
		// Search local extremum
		int argmax = absArgmax(scanline);
		int maxAbsValue = abs(scanline[argmax]);
		while (maxAbsValue >= INTENSITY_THRESHOLD) {

			cv::Vec2i position = argmax * scanDir + strideIdx * strideDir;
			int channel1Val = MathTools::convolution(frame, frameSize, filter, position, scanDir, 1);
			int channel2Val = MathTools::convolution(frame, frameSize, filter, position, scanDir, 2);

			if (abs(scanline[argmax] - channel1Val) < CHANNEL_GAP_THRESHOLD
				&& abs(scanline[argmax] - channel2Val) < CHANNEL_GAP_THRESHOLD)
			{
				// Create new edgel
				//TODO: ORIENTATION
				//TODO: NO DUPLICATE
				addEdgel(position, 0.0f, type);
			}

			nullifyNeighbors(scanline, argmax);
			argmax = absArgmax(scanline);
			maxAbsValue = abs(scanline[argmax]);
		}

	}
}

int FrameProcessing::absArgmax(std::vector<int>& scanline)
{
	int argmax = 0;
	int max = abs(scanline[0]);

	for (int k = 1; k < scanline.size(); k++) {
		if (abs(scanline[k]) > max) {
			argmax = k;
			max = abs(scanline[k]);
		}
	}

	return argmax;
}

void FrameProcessing::nullifyNeighbors(std::vector<int>& scanline, int index)
{
	for (int k = std::max(0, index - NEIGHBORHOOD_SIZE / 2); k < std::min((int)scanline.size(), index + NEIGHBORHOOD_SIZE / 2); k++) {
		scanline[k] = 0;
	}
}