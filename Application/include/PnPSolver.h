#pragma once

#include <opencv2/opencv.hpp>
#include <vector>


class PnPSolver {
public:
	PnPSolver(int width, int height);
	~PnPSolver();
	void solve(std::vector<cv::Vec2i> corners);

private:
	void computeControlPoints();
	void computeBarycentricCoords();

	//DATA
	const int uc;
	const int vc;
	cv::Vec3d pw[4];
	cv::Vec3d cw[3];
	cv::Mat alpha;
};