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
	void execute(cv::Mat & frame);
	cv::Vec2i getRegionOrigin();
	cv::Vec2i getRegionNumber();
	std::vector<Edgel> getEdgelList();
	std::vector<Line> getLineList();
	std::vector<Line> getMergedLineList();

private:
	void reinitNeededRegions();
	void addEdgel(cv::Vec2i position, float orientation, EdgelType type);
	void findEdgels(cv::Mat & frame);
	void scanLines(cv::Mat & frame, cv::Vec2i scanDir, EdgelType type);
	void getAbsArgmaxList(std::vector<int> &argList, std::vector<int> &scanline);
	void RANSACGrouper();
	void initEdgelsList(std::vector<int> & index, int i, int j);
	HypoLine getHypotheticLine(std::vector<int> & index, std::vector<Edgel> & edgels);
	int countCompatibleEdgels(HypoLine & line, std::vector<int> & index, std::vector<Edgel> & edgels);
	void mergeLines();
	void addMergedLines(std::vector<Line> & finalLineList, std::vector<Line> & initialLineList);

	// DATA
	cv::Vec2i frameSize;
	cv::Vec2i regionOrigin;
	cv::Vec2i regionNumber;
	std::vector<std::vector<Region>> regionGrid;
	std::vector<Line> mergedLines;
};