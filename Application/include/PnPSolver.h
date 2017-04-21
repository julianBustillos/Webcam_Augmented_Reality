#pragma once

#include <opencv2/opencv.hpp>
#include <vector>


class PnPSolver {
public:
	PnPSolver(int width, int height);
	~PnPSolver();
	void solve(std::vector<cv::Vec2i> corners);
	cv::Vec3d getPointCameraCoords(cv::Vec3d point) const;

private:
	void computeFocalLength();
	void computeControlPoints();
	void computeBarycentricCoords();
	void fillM();
	void computeMNullSpace();
	void computeBeta();
	void computePC();
	void estimateTransformation();
	double getMeanReprojectionError() const;

	//DATA
	double f[2];
	int uc[2];
	cv::Vec3d pw[4];
	cv::Vec3d pc[4];
	cv::Vec3d cw[3];
	cv::Vec2i u[4];
	cv::Mat alpha;
	cv::Mat M;
	cv::Mat vDist;
	cv::Mat cwDist;
	cv::Vec3d v[3];
	double beta;
	double c;
	cv::Mat R;
	cv::Mat t;
};