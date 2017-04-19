#pragma once

#include <opencv2/opencv.hpp>
#include <vector>


class PnPSolver {
public:
	PnPSolver(int width, int height);
	~PnPSolver();
	void solve(std::vector<cv::Vec2i> corners);

private:
	void computeFocalLength();
	void computeControlPoints();
	void computeBarycentricCoords();
	void fillM();
	void computeMNullSpace();

	//DATA
	double f[2];
	int uc[2];
	cv::Vec3d pw[4];
	cv::Vec3d cw[3];
	cv::Vec2i u[4];
	cv::Mat alpha;
	cv::Mat M;
	cv::Mat W;
	cv::Mat U;
	cv::Mat Vt;
	cv::Mat eigenvalues;
	cv::Mat eigenvectors;
	cv::Mat v;
};