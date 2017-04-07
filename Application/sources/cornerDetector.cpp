#include "cornerDetector.h"
#include "debug.h"
#include "constants.h"
#include <iostream>
#include "mathTools.h"
#include <chrono>

#ifdef DEBUG
#include <time.h>
#endif


CornerDetector::CornerDetector(int width, int height) :
	lastExecTime(0.0)
{
	frameSize[0] = height;
	frameSize[1] = width;
	regionOrigin[0] = 0;
	regionOrigin[1] = 0;
	regionNumber[0] = (int)ceil((float)height / GET(REGION_PIXEL_SIZE));
	regionNumber[1] = (int)ceil((float)width / GET(REGION_PIXEL_SIZE));
	
	regionGrid.resize(regionNumber[0]);
	for (int i = 0; i < regionGrid.size(); i++) {
		regionGrid[i].resize(regionNumber[1]);
	}

	std::cout << "Region grid size : " << regionNumber[0] << "*" << regionNumber[1] << std::endl;
}

CornerDetector::~CornerDetector()
{
}

void CornerDetector::execute(const cv::Mat & frame)
{
#ifdef DEBUG
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
#endif

	findEdgels(frame);
	RANSACGrouper();
	mergeLines(frame);
	extendLines(frame);

#ifdef DEBUG
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	lastExecTime = std::chrono::duration<double>(end - start).count();
#endif
}

const cv::Vec2i CornerDetector::getRegionOrigin() const
{
	return regionOrigin;
}

const cv::Vec2i CornerDetector::getRegionNumber() const
{
	return regionNumber;
}

const std::vector<Line> CornerDetector::getMergedLineList() const
{
	return mergedLines;
}

const std::vector<Line> CornerDetector::getExtendedLineList() const
{
	return extendedLines;
}

const std::vector<Edgel> CornerDetector::getEdgelList() const
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

const std::vector<Line> CornerDetector::getLineList() const
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

void CornerDetector::reinitNeededRegions()
{
	for (int i = 0; i < regionNumber[0]; i++) {
		for (int j = 0; j < regionNumber[1]; j++) {
			regionGrid[i][j].edgels.clear();
			regionGrid[i][j].lines.clear();
		}
	}
}

void CornerDetector::addEdgel(cv::Vec2i position, float orientation, EdgelType type)
{
	Edgel edgel;
	edgel.position = position;
	edgel.orientation = orientation;
	edgel.type = type;

	cv::Vec2i regionIndex;
	regionIndex[0] = (int)floor((float)(position[0]-regionOrigin[0]) / GET(REGION_PIXEL_SIZE));
	regionIndex[1] = (int)floor((float)(position[1]-regionOrigin[1]) / GET(REGION_PIXEL_SIZE));

	regionGrid[regionIndex[0]][regionIndex[1]].edgels.push_back(edgel);
}

void CornerDetector::findEdgels(const cv::Mat & frame)
{
	reinitNeededRegions();
	cv::Vec2i verticalScanDir(1, 0);
	scanLines(frame, verticalScanDir, EdgelType::horizontal);
	cv::Vec2i horizontalScanDir(0, 1);
	scanLines(frame, horizontalScanDir, EdgelType::vertical);
}

void CornerDetector::scanLines(const cv::Mat & frame, cv::Vec2i scanDir, EdgelType type)
{
	cv::Vec2i strideDir = cv::Vec2i(1, 1) - scanDir;
	int strideIdxFirst = regionOrigin.dot(strideDir);
	int strideIdxLast = std::min(strideIdxFirst + regionNumber.dot(strideDir) * GET(REGION_PIXEL_SIZE), frameSize.dot(strideDir));

	int scanIdxFirst = regionOrigin.dot(scanDir);
	int scanIdxLast = std::min(scanIdxFirst + regionNumber.dot(scanDir) * GET(REGION_PIXEL_SIZE), frameSize.dot(scanDir));
	std::vector<int> scanline(scanIdxLast - scanIdxFirst);

	std::vector<int> filter = GET(FILTER);
	std::vector<int> argList;

	for (int strideIdx = strideIdxFirst; strideIdx < strideIdxLast; strideIdx += GET(SCANLINE_STRIDE)) {
		for (int scanIdx = 0; scanIdx < scanline.size(); scanIdx++) {

			// Get all scanline values
			cv::Vec2i position = scanIdx * scanDir + strideIdx * strideDir;
			scanline[scanIdx] = MathTools::convolution(frame, frameSize, filter, position, scanDir, 0);

		}
			
		// Search local extremum
		getAbsArgmaxList(argList, scanline);
		for (int argIdx = 0; argIdx < argList.size(); argIdx++) {

			cv::Vec2i position = argList[argIdx] * scanDir + strideIdx * strideDir;
			int channel1Val = MathTools::convolution(frame, frameSize, filter, position, scanDir, 1);
			int channel2Val = MathTools::convolution(frame, frameSize, filter, position, scanDir, 2);

			if (abs(scanline[argList[argIdx]] - channel1Val) < GET(CHANNEL_GAP_THRESHOLD)
				&& abs(scanline[argList[argIdx]] - channel2Val) < GET(CHANNEL_GAP_THRESHOLD)) {
				// Create new edgel
				int strideDirVal = MathTools::convolution(frame, frameSize, filter, position, strideDir, 0);
				float orientation = MathTools::edgelOrientation(scanline[argList[argIdx]], strideDirVal, type);
				addEdgel(position, orientation, type);
			}
		}

	}
}

void CornerDetector::getAbsArgmaxList(std::vector<int>& argList, const std::vector<int>& scanline) const
{
	int currentArgmax = -1;
	int currentMax = -1;
	int temp;
	argList.clear();

	for (int k = 0; k < scanline.size(); k++) {
		temp = abs(scanline[k]);
		if (temp <= GET(INTENSITY_THRESHOLD)) {
			if (currentArgmax >= 0) {
				argList.push_back(currentArgmax);
				currentMax = -1;
				currentArgmax = -1;
			}
		}
		else if (temp > currentMax) {
			currentMax = temp;
			currentArgmax = k;
		}
	}
}

void CornerDetector::RANSACGrouper()
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

			while (++iterations < GET(MAX_LINE_SEARCH_ITER) && index.size() > 2) {

				// Search dominant line
				maxVotes = -1;

				for (int id = 0; id < GET(DOMINANT_LINE_SEARCH_ATTEMPTS); id++) {
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
				if (maxVotes >= GET(MIN_DOMINANT_LINE_VOTES)) {
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

void CornerDetector::initEdgelsList(std::vector<int>& index, int i, int j)
{
	index.resize(regionGrid[i][j].edgels.size());

	for (int k = 0; k < index.size(); k++) {
		index[k] = k;
	}
}

HypoLine CornerDetector::getHypotheticLine(const std::vector<int>& index, const std::vector<Edgel> & edgels) const
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
		lineOr = MathTools::lineOrientation(edgels[index[id1]].position, edgels[index[id2]].position);
		
		float diffL1 = MathTools::orientationDiff(lineOr, or1);
		float diffL2 = MathTools::orientationDiff(lineOr, or2);
		
		oriOK = (diffL1 < GET(ORIENTATION_TOLERANCE)) && (diffL2 < GET(ORIENTATION_TOLERANCE));

		if (count++ > GET(HYPOLINE_ATTEMPTS)) {
			line.id1 = -1;
			return line;
		}
	}

	line.id1 = id1;
	line.id2 = id2;
	line.orientation = lineOr;

	return line;
}

int CornerDetector::countCompatibleEdgels(HypoLine & line, const std::vector<int>& index, const std::vector<Edgel>& edgels) const
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
		if (abs(orDiff) > GET(ORIENTATION_TOLERANCE)) {
			line.nonVotersId.push_back(idx);
			continue;
		}

		dist = MathTools::pointLineDistance(edgels[index[line.id1]].position, edgels[index[line.id2]].position, edgels[index[idx]].position);
		if (dist > GET(POINT_LINE_DIST_TOLERANCE)) {
			line.nonVotersId.push_back(idx);
			continue;
		}

		count++;
	}

	return count;
}

void CornerDetector::mergeLines(const cv::Mat & frame)
{
	std::vector<Line> regionMergedLines;
	regionMergedLines.clear();
	mergedLines.clear();
	std::vector<int> filter = GET(FILTER);

	// Merge lines by region
	for (int i = 0; i < regionNumber[0]; i++) {
		for (int j = 0; j < regionNumber[1]; j++) {
			addMergedLines(frame, filter, regionMergedLines, regionGrid[i][j].lines);
		}
	}

	// Merge all lines
	addMergedLines(frame, filter, mergedLines, regionMergedLines);
}

bool CornerDetector::compatibleOrientation(Line & l1, Line & l2) const {
	return (MathTools::orientationDiff(l1.orientation, l2.orientation) < GET(ORIENTATION_TOLERANCE));
}

bool CornerDetector::compatibleConnectionOrientation(Line & l1, Line & l2, Merge & merge) const {
	merge.dist = MathTools::pointPointDistance(l1.p2, l2.p1);
	merge.ext1 = l1.p1;
	merge.merge1 = l1.p2;
	merge.merge2 = l2.p1;
	merge.ext2 = l2.p2;
	float temp;

	temp = MathTools::pointPointDistance(l1.p1, l2.p2);
	if (temp < merge.dist) {
		merge.dist = temp;
		merge.ext1 = l2.p1;
		merge.merge1 = l2.p2;
		merge.merge2 = l1.p1;
		merge.ext2 = l1.p2;
	}

	float lineOri = MathTools::lineOrientation(merge.merge1, merge.merge2);

	return (MathTools::orientationDiff(l1.orientation, lineOri) < GET(ORIENTATION_TOLERANCE)) && (MathTools::orientationDiff(l2.orientation, lineOri) < GET(ORIENTATION_TOLERANCE));
}

bool CornerDetector::compatibleConnectionPixelsOrientation(const cv::Mat & frame, std::vector<int> & filter, Merge & merge) const {
	merge.orientation = MathTools::lineOrientation(merge.ext1, merge.ext2);
	int X = merge.merge1[0];
	int Y = merge.merge1[1];
	float tDeltaX, tDeltaY;
	float tMaxX, tMaxY;
	int stepX, stepY;
	int convolution[2];
	float orientation;

	if (merge.merge2[0] == X) {
		tDeltaX = 0;
	}
	else {
		tDeltaX = 1.0f / abs(merge.merge2[0] - X);
	}
	tMaxX = tDeltaX / 2.0f;

	if (merge.merge2[1] == Y) {
		tDeltaY = 0;
	}
	else {
		tDeltaY = 1.0f / abs(merge.merge2[1] - Y);
	}
	tMaxY = tDeltaY / 2.0f;

	if (merge.merge2[0] > X) {
		stepX = 1;
	}
	else {
		stepX = -1;
	}
	if (merge.merge2[1] > Y) {
		stepY = 1;
	}
	else {
		stepY = -1;
	}

	while (true) {
		if (tMaxX < tMaxY) {
			tMaxX += tDeltaX;
			X += stepX;
		}
		else {
			tMaxY += tDeltaY;
			Y += stepY;
		}

		if (X == merge.merge2[0] && Y == merge.merge2[1]) {
			break;
		}

		convolution[0] = MathTools::convolution(frame, frameSize, filter, cv::Vec2i(X, Y), cv::Vec2i(1, 0), 0);
		convolution[1] = MathTools::convolution(frame, frameSize, filter, cv::Vec2i(X, Y), cv::Vec2i(0, 1), 0);
		orientation = MathTools::edgelOrientation(convolution[0], convolution[1], EdgelType::horizontal);
		if (MathTools::orientationDiff(orientation, merge.orientation) > GET(ORIENTATION_TOLERANCE)) {
			return false;
		}
	}

	return true;
}

void CornerDetector::deleteMergedLines(std::vector<Line>& lineList, int l1Idx, int l2Idx) const
{
	int minIdx = std::min(l1Idx, l2Idx);
	int maxIdx = std::max(l1Idx, l2Idx);
	int next = 1;

	for (int idx = minIdx; idx < lineList.size() - 2; idx++) {
		if (idx + next == maxIdx) {
			next++;
		}
		lineList[idx] = lineList[idx + next];
	}
	
	lineList.pop_back();
	lineList.pop_back();
}

void CornerDetector::addMergedLines(const cv::Mat & frame, std::vector<int> & filter, std::vector<Line>& finalLineList, std::vector<Line>& initialLineList) const
{
	Merge merge;
	Line newLine;
	std::vector<Merge> merges;
	bool finished = false;
	std::vector<Line> temp = initialLineList;

	while (!finished) {
		merges.clear();

		// Search merge possibilities (with 2 criterions)
		for (int l1Idx = 0; l1Idx < temp.size(); l1Idx++) {
			for (int l2Idx = l1Idx + 1; l2Idx < temp.size(); l2Idx++) {
				if (compatibleOrientation(temp[l1Idx], temp[l2Idx]) && compatibleConnectionOrientation(temp[l1Idx], temp[l2Idx], merge)) {
					merge.l1Idx = l1Idx;
					merge.l2Idx = l2Idx;
					merges.push_back(merge);
				}
			}
		}

		// Sort merges by extremities distances
		std::sort(merges.begin(), merges.end());

		// Find and apply a merging operation
		finished = true;
		for (int mIdx = 0; mIdx < merges.size(); mIdx++) {
			if (compatibleConnectionPixelsOrientation(frame, filter, merges[mIdx])) {
				deleteMergedLines(temp, merges[mIdx].l1Idx, merges[mIdx].l2Idx);
				newLine.p1 = merges[mIdx].ext1;
				newLine.p2 = merges[mIdx].ext2;
				newLine.orientation = merges[mIdx].orientation;
				temp.push_back(newLine);
				finished = false;
				break;
			}
		}

	}

	// Complete final list with new merged lines
	for (int mlIdx = 0; mlIdx < temp.size(); mlIdx++) {
		finalLineList.push_back(temp[mlIdx]);
	}
}

void CornerDetector::extendLines(const cv::Mat & frame)
{
	extendedLines.clear();
	//TODO
}

double CornerDetector::getLastExecTime() const
{
	return lastExecTime;
}
