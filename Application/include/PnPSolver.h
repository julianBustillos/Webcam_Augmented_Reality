#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <glm/glm.hpp>
#include "LevenbergMarquardt.h"


class PnPSolver {
public:
	PnPSolver(int width, int height);
	~PnPSolver();
	void solve(const std::vector<cv::Vec2i> & corners, bool identified);
	glm::vec3 getCameraPosition() const;
	glm::vec3 getCameraFront() const;
	glm::vec3 getCameraUp() const;
	bool wasSolvedLastFrame() const;

private:
	void computeFocalLength();
	void estimatePlaneTransformation();
	void homographyInit();
	double getMeanReprojectionError() const;
	cv::Mat getPointCameraCoords(cv::Vec3d point) const;
	cv::Vec3d getPointWorldCoords(cv::Vec3d point) const;
	glm::vec3 worldToOpenGLCoords(cv::Vec3d point) const;

	//DATA
	bool firstFrameSolve;
	bool solvedLastFrame;

	LevenbergMarquardt lm;

	cv::Mat A;
	cv::Mat A_inv;
	std::vector<cv::Vec3d> pw;
	std::vector<cv::Vec2d> pw_plane;
	std::vector<cv::Vec2d> u_n;
	cv::Mat R_plane;
	cv::Mat t_plane;
	cv::Mat homography;
	cv::Mat R;
	cv::Mat R_inv;
	cv::Mat t;
};