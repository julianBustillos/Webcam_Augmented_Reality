#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry.h"
#include <deque>


class CornerDetector {
public:
	CornerDetector(int width, int height);
	~CornerDetector();
	void execute(const cv::Mat & frame);
	const cv::Vec2i getRegionOrigin() const;
	const cv::Vec2i getRegionNumber() const;
	const std::vector<Edgel> getEdgelList() const;
	const std::vector<Line> getLineList() const;
	const std::vector<Line> getMergedLineList() const;
	const std::vector<Line> getExtendedLineList() const;
	const std::vector<std::vector<cv::Vec2i>> getCornerGroupsList() const;
	void setROI(const std::vector<cv::Vec2i> & ROI);
	double getLastExecTime() const;

private:
	void reinitNeededRegions();
	void addEdgel(cv::Vec2i position, float orientation, EdgelType type);
	void findEdgels(const cv::Mat & frame);
	void scanLines(const cv::Mat & frame, cv::Vec2i scanDir, EdgelType type);
	void getAbsArgmaxList(std::vector<int> &argList, const std::vector<int> &scanline) const;
	void RANSACGrouper();
	void initEdgelsList(std::vector<int> & index, int i, int j);
	HypoLine getHypotheticLine(const std::vector<int> & index, const std::vector<Edgel> & edgels) const;
	int countCompatibleEdgels(HypoLine & line, const std::vector<int> & index, const std::vector<Edgel> & edgels) const;
	void mergeLines(const cv::Mat & frame);
	bool compatibleOrientation(Line & l1, Line & l2) const;
	bool compatibleConnectionOrientation(Line & l1, Line & l2, Merge & merge) const;
	void initRayTracing(const cv::Vec2i & start, const cv::Vec2i & dir, int & X, int & Y, float & tDeltaX, float & tDeltaY, float & tMaxX, float & tMaxY, int & stepX, int & stepY) const;
	bool compatibleConnectionPixelsOrientation(const cv::Mat & frame, std::vector<int> & filter, Merge & merge) const;
	float getPointOrientation(const cv::Mat & frame, const std::vector<int> & filter, const cv::Vec2i & point, int channel) const;
	void deleteMergedLines(std::vector<Line> & lineList, int l1Idx, int l2Idx) const;
	void addMergedLines(const cv::Mat & frame, std::vector<int> & filter, std::vector<Line>& finalLineList, std::vector<Line>& initialLineList) const;
	void extendLines(const cv::Mat & frame);
	void getExtremity(const cv::Mat & frame, std::vector<int> & filter, cv::Vec2i & start, const cv::Vec2i & dir, float lineOrientation) const;
	bool isValid(const cv::Mat & frame, std::vector<int> & filter, const cv::Vec2i & point, const cv::Vec2i & dir);
	void detectCorners();
	void getQuadrangle(std::deque<int> & indexList, const std::vector<int> & availableLines) const;
	void removeLines(const std::deque<int> & indexList, std::vector<int> & availableLines) const;
	void getCorners(std::vector<cv::Vec2i> & corners, const std::deque<int> quadLines) const;
	bool isNextLine(const cv::Vec2i & p1, const cv::Vec2i & p2, float ori1, float ori2) const;
	void getLineGroup(std::deque<int> & lineGroup, const std::deque<int> & indexList, const std::vector<int> & availableLines) const;

	// DATA
	double lastExecTime;
	cv::Vec2i frameSize;
	cv::Vec2i regionOrigin;
	cv::Vec2i regionNumber;
	std::vector<std::vector<Region>> regionGrid;
	std::vector<Line> mergedLines;
	std::vector<Line> extendedLines;
	std::vector<std::vector<cv::Vec2i>> cornerGroups;
};