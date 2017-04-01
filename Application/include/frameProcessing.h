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

struct Region {
	std::vector<Edgel> edgels;
};

class FrameProcessing {
public:
	FrameProcessing(int width, int height);
	~FrameProcessing();
	void execute(cv::Mat & frame);
	std::vector<Edgel> getEdgelList();

private:
	cv::Vec2i frameSize;
	cv::Vec2i regionOrigin;
	cv::Vec2i regionNumber;
	std::vector<std::vector<Region>> regionGrid;
	void reinitNeededRegions();
	void addEdgel(cv::Vec2i position, float orientation, EdgelType type);
	void findEdgels(cv::Mat & frame);
	void scanLines(cv::Mat & frame, cv::Vec2i scanDir, EdgelType type);
	int absArgmax(std::vector<int> &scanline);
	void nullifyNeighbors(std::vector<int> &scanline, int index);
};