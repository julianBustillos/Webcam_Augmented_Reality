#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "geometry.h"


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
	bool compatibleConnectionPixelsOrientation(const cv::Mat & frame, std::vector<int> & filter, Merge & merge) const;
	void deleteMergedLines(std::vector<Line> & lineList, int l1Idx, int l2Idx) const;
	void addMergedLines(const cv::Mat & frame, std::vector<int> & filter, std::vector<Line>& finalLineList, std::vector<Line>& initialLineList) const;
	void extendLines(const cv::Mat & frame);

	// DATA
	double lastExecTime;
	cv::Vec2i frameSize;
	cv::Vec2i regionOrigin;
	cv::Vec2i regionNumber;
	std::vector<std::vector<Region>> regionGrid;
	std::vector<Line> mergedLines;
	std::vector<Line> extendedLines;
};