#pragma once

#include <vector>
#include <opencv2/opencv.hpp>


enum class EdgelType {
	horizontal,
	vertical
};

struct Edgel {
	cv::Vec2i position;
	float orientation;
	EdgelType type;
};

struct Line {
	cv::Vec2i p1;
	cv::Vec2i p2;
	float orientation;
};

struct HypoLine {
	int id1;
	int id2;
	float orientation;
	std::vector<int> nonVotersId;
};

struct Region {
	std::vector<Edgel> edgels;
	std::vector<Line> lines;
};

class FrameProcessing {
public:
	FrameProcessing(int width, int height);
	~FrameProcessing();
	void execute(const cv::Mat & frame);
	const cv::Vec2i getRegionOrigin() const;
	const cv::Vec2i getRegionNumber() const;
	const std::vector<Edgel> getEdgelList() const;
	const std::vector<Line> getLineList() const;
	const std::vector<Line> getMergedLineList() const;

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
	void mergeLines();
	void addMergedLines(std::vector<Line> & finalLineList, const std::vector<Line> & initialLineList) const;

	// DATA
	cv::Vec2i frameSize;
	cv::Vec2i regionOrigin;
	cv::Vec2i regionNumber;
	std::vector<std::vector<Region>> regionGrid;
	std::vector<Line> mergedLines;
};