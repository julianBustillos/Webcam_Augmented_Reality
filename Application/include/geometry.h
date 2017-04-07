#pragma once

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
	bool isValid;
};

struct HypoLine {
	int id1;
	int id2;
	float orientation;
	std::vector<int> nonVotersId;
};

struct Merge {
	int l1Idx;
	int l2Idx;
	cv::Vec2i ext1;
	cv::Vec2i merge1;
	cv::Vec2i merge2;
	cv::Vec2i ext2;
	float dist;
	float orientation;

	bool operator < (const Merge & merge) const
	{
		return (dist < merge.dist);
	}
};

struct Region {
	std::vector<Edgel> edgels;
	std::vector<Line> lines;
};