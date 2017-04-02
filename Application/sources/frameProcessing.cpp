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
	RANSACGrouper();
}

cv::Vec2i FrameProcessing::getRegionOrigin()
{
	return regionOrigin;
}

cv::Vec2i FrameProcessing::getRegionNumber()
{
	return regionNumber;
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

std::vector<Line> FrameProcessing::getLineList()
{
	std::vector<Line> lineList;
	lineList.clear();

	for (int i = 0; i < regionNumber[0]; i++) {
		for (int j = 0; j < regionNumber[1]; j++) {
			for (int k = 0; k < regionGrid[i][j].lines.size(); k++) {
				lineList.push_back(regionGrid[i][j].lines[k]);
			}
		}
	}

	return lineList;
}

void FrameProcessing::reinitNeededRegions()
{
	for (int i = 0; i < regionNumber[0]; i++) {
		for (int j = 0; j < regionNumber[1]; j++) {
			regionGrid[i][j].edgels.clear();
			regionGrid[i][j].lines.clear();
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
				int strideDirVal = MathTools::convolution(frame, frameSize, filter, position, strideDir, 0);
				float orientation;
				if (type == EdgelType::vertical) {
					orientation = std::atan2f((float)scanline[argmax], (float)strideDirVal);
				}
				else {
					orientation = std::atan2f((float)strideDirVal, (float)scanline[argmax]);
				}

				addEdgel(position, orientation, type);
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

void FrameProcessing::RANSACGrouper()
{
	std::vector<int> index;
	HypoLine dominantLine;
	HypoLine line;
	int maxVotes;
	int iterations;

	for (int i = 0; i < regionNumber[0]; i++) {
		for (int j = 0; j < regionNumber[1]; j++) {

			initEdgelsList(index, i, j);
			iterations = 0;

			while (++iterations < MAX_LINE_SEARCH_ITER && index.size() > 2) {

				// Search dominant line
				maxVotes = -1;

				for (int id = 0; id < DOMINANT_LINE_SEARCH_ATTEMPTS; id++) {
					line = getHypotheticLine(index, regionGrid[i][j].edgels);
					if (line.id1 < 0) {
						break;
					}

					int temp = countCompatibleEdgels(line, index, regionGrid[i][j].edgels);
					if (temp > maxVotes) {
						dominantLine = line;
						maxVotes = temp;
					}
				}

				// Add dominant line and remove edgels OR no dominant line
				if (maxVotes >= MIN_DOMINANT_LINE_VOTES) {
					Line newLine;
					newLine.p1 = regionGrid[i][j].edgels[index[dominantLine.id1]].position;
					newLine.p2 = regionGrid[i][j].edgels[index[dominantLine.id2]].position;
					newLine.orientation = dominantLine.orientation;

					regionGrid[i][j].lines.push_back(newLine);

					index = dominantLine.nonVotersId;
				}
				else {
					break;
				}
			}
		}
	}
}

void FrameProcessing::initEdgelsList(std::vector<int>& index, int i, int j)
{
	index.resize(regionGrid[i][j].edgels.size());

	for (int k = 0; k < index.size(); k++) {
		index[k] = k;
	}
}

HypoLine FrameProcessing::getHypotheticLine(std::vector<int>& index, std::vector<Edgel> & edgels)
{
	int id1 = 0, id2 = 0;
	float or1 = 0.0f, or2 = 0.0f;
	float lineOr = 0.0f;
	bool oriOK = false;
	int count = 0;
	HypoLine line;

	while (id1 == id2 || !oriOK) {
		        
		id1 = MathTools::random((int)index.size());
		id2 = MathTools::random((int)index.size());

		or1 = edgels[index[id1]].orientation;
		or2 = edgels[index[id2]].orientation;
		lineOr = MathTools::lineOrientations(edgels[index[id1]].position, edgels[index[id2]].position);
		
		float diffL1 = MathTools::orientationDiff(lineOr, or1);
		float diffL2 = MathTools::orientationDiff(lineOr, or2);
		
		oriOK = (diffL1 < ORIENTATION_TOLERANCE) && (diffL2 < ORIENTATION_TOLERANCE);

		if (count++ >= HYPOLINE_ATTEMPTS) {
			line.id1 = -1;
			return line;
		}
	}

	line.id1 = id1;
	line.id2 = id2;
	line.orientation = lineOr;

	return line;
}

int FrameProcessing::countCompatibleEdgels(HypoLine & line, std::vector<int>& index, std::vector<Edgel>& edgels)
{
	line.nonVotersId.clear();
	int count = 0;
	float dist;
	float orDiff;

	for (int idx = 0; idx < index.size(); idx++) {
		if (idx == line.id1 || idx == line.id2) {
			line.nonVotersId.push_back(idx);
			continue;
		}
		
		orDiff = MathTools::orientationDiff(line.orientation, edgels[index[idx]].orientation);
		if (abs(orDiff) > ORIENTATION_TOLERANCE) {
			line.nonVotersId.push_back(idx);
			continue;
		}

		dist = MathTools::pointLineDistance(edgels[index[line.id1]].position, edgels[index[line.id2]].position, edgels[index[idx]].position);
		if (dist > POINT_LINE_DIST_TOLERANCE) {
			line.nonVotersId.push_back(idx);
			continue;
		}

		count++;
	}

	return count;
}
