#include "PnPSolver.h"
#include "constants.h"
#include <chrono>
#include "mathTools.h"


PnPSolver::PnPSolver(int width, int height) :
	solvedLastFrame(false), firstFrameSolve(false)
{
	A = cv::Mat::zeros(3, 3, CV_64F);
	A.at<double>(0, 2) = height / 2.;
	A.at<double>(1, 2) = width / 2.;
	A.at<double>(2, 2) = 1.;

	pw.resize(4);
	pw[0] = cv::Vec3d(0.,  0.,  0.);
	pw[1] = cv::Vec3d(-10.,  0.,  0.);
	pw[2] = cv::Vec3d(-10.,  0., -10.);
	pw[3] = cv::Vec3d( 0.,  0., -10.);

	pw_plane.resize(4);
	u_n.resize(4);

	R_plane = cv::Mat::zeros(3, 3, CV_64F);
	t_plane = cv::Mat::zeros(3, 1, CV_64F);

	R = cv::Mat::zeros(3, 3, CV_64F);
	R_inv = cv::Mat::zeros(3, 3, CV_64F);
	t = cv::Mat::zeros(3, 1, CV_64F);

	computeFocalLength();
	A_inv = A.inv(cv::DECOMP_SVD);

	estimatePlaneTransformation();

	lm.setPW(pw);
}

PnPSolver::~PnPSolver()
{
}

void PnPSolver::solve(const std::vector<cv::Vec2i> & corners, bool identified)
{
	if (!identified) {
		solvedLastFrame = false;
		firstFrameSolve = false;
		return;
	}

	for (int idx = 0; idx < 4; idx++) {
		u_n[idx][0] = A_inv.at<double>(0, 0) * corners[idx][0] + A_inv.at<double>(0, 1) * corners[idx][1] + A_inv.at<double>(0, 2);
		u_n[idx][1] = A_inv.at<double>(1, 0) * corners[idx][0] + A_inv.at<double>(1, 1) * corners[idx][1] + A_inv.at<double>(1, 2);
		double w    = A_inv.at<double>(2, 0) * corners[idx][0] + A_inv.at<double>(2, 1) * corners[idx][1] + A_inv.at<double>(2, 2);
		u_n[idx] /= w;
	}

	if (firstFrameSolve) {
		solvedLastFrame = true;
	}
	else {
		firstFrameSolve = true;
		homographyInit();
	}

	lm.init(u_n, R, t);
	lm.optimize();
	lm.getResult(R, t);
	
	R_inv = R.inv(cv::DECOMP_SVD);
}

glm::vec3 PnPSolver::getCameraPosition() const
{
	cv::Vec3d cameraPositionC = cv::Vec3d(0.0f, 0.0f, 0.0f);
	cv::Vec3d cameraPositionW = getPointWorldCoords(cameraPositionC);
	
	return worldToOpenGLCoords(cameraPositionW);
}

glm::vec3 PnPSolver::getCameraFront() const
{
	cv::Vec3d cameraPositionC = cv::Vec3d(0.0f, 0.0f, 0.0f);
	cv::Vec3d cameraPositionW = getPointWorldCoords(cameraPositionC);
	cv::Vec3d cameraFrontC = cv::Vec3d(0.0f, 0.0f, 1.0f);
	cv::Vec3d cameraFrontW = getPointWorldCoords(cameraFrontC);
	glm::vec3 cameraFrontRes = glm::normalize(worldToOpenGLCoords(cameraFrontW) - worldToOpenGLCoords(cameraPositionW));

	return cameraFrontRes;
}

glm::vec3 PnPSolver::getCameraUp() const
{
	cv::Vec3d cameraPositionC = cv::Vec3d(0.0f, 0.0f, 0.0f);
	cv::Vec3d cameraPositionW = getPointWorldCoords(cameraPositionC);
	cv::Vec3d cameraUpC = cv::Vec3d(-1.0f, 0.0f, 0.0f);
	cv::Vec3d cameraUpW = getPointWorldCoords(cameraUpC);
	glm::vec3 cameraUpRes = glm::normalize(worldToOpenGLCoords(cameraUpW) - worldToOpenGLCoords(cameraPositionW));

	return cameraUpRes;
}

void PnPSolver::computeFocalLength()
{
	A.at<double>(0, 0) = A.at<double>(1, 1) = A.at<double>(1, 2) / tan(GET(HFOV) * M_PI / 360);
}

void PnPSolver::estimatePlaneTransformation()
{
	cv::Mat mu = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat L = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat Sigma = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat U;
	cv::Mat W;

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 3; i++) {
			mu.at<double>(i, 0) += pw[j][i];
		}
	}
	mu /= 4.;

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 3; i++) {
			L.at<double>(i, 0) = pw[j][i] - mu.at<double>(i, 0);
		}
		Sigma += L * L.t();
	}

	cv::SVD::compute(Sigma, W, U, R_plane);

	if (MathTools::det3x3(R_plane) < 0) {
		R_plane *= -1.;
	}

	t_plane = - R_plane * mu;

	for (int idx = 0; idx < 4; idx++) {
		pw_plane[idx][0] = R_plane.at<double>(0, 0) * pw[idx][0] + R_plane.at<double>(0, 1) * pw[idx][1] + R_plane.at<double>(0, 2) * pw[idx][2] + t_plane.at<double>(0, 0);
		pw_plane[idx][1] = R_plane.at<double>(1, 0) * pw[idx][0] + R_plane.at<double>(1, 1) * pw[idx][1] + R_plane.at<double>(1, 2) * pw[idx][2] + t_plane.at<double>(1, 0);
	}
}

void PnPSolver::homographyInit()
{
	MathTools::findHomography(u_n, pw_plane, homography);

	double h_norm[2];
	h_norm[0] = MathTools::norm3x1(homography.col(0));
	h_norm[1] = MathTools::norm3x1(homography.col(1));

	for (int i = 0; i < 3; i++) {
		homography.at<double>(i, 0) /= h_norm[0];
		homography.at<double>(i, 1) /= h_norm[1];
	}

	t = homography.col(2) * 2. / (h_norm[0] + h_norm[1]);

	homography.at<double>(0, 2) = homography.at<double>(1, 0) * homography.at<double>(2, 1) - homography.at<double>(2, 0) * homography.at<double>(1, 1);
	homography.at<double>(1, 2) = homography.at<double>(2, 0) * homography.at<double>(0, 1) - homography.at<double>(0, 0) * homography.at<double>(2, 1);
	homography.at<double>(2, 2) = homography.at<double>(0, 0) * homography.at<double>(1, 1) - homography.at<double>(1, 0) * homography.at<double>(0, 1);

	t += homography * t_plane;
	R = homography * R_plane;
}

double PnPSolver::getMeanReprojectionError() const
{
	double error = 0.;

	for (int i = 0; i < 4; i++) {
		cv::Vec3d ui_n = cv::Vec3d(u_n[i][0], u_n[i][1], 1.);
		cv::Mat pwi_c = getPointCameraCoords(pw[i]);
		pwi_c /= pwi_c.at<double>(2, 0);

		error += MathTools::diffSquareNorm(ui_n, pwi_c);
	}

	return error / 4;
}

cv::Mat PnPSolver::getPointCameraCoords(cv::Vec3d point) const
{
	cv::Mat pointW = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat pointC = cv::Mat::zeros(3, 1, CV_64F);

	pointW.at<double>(0, 0) = point[0];
	pointW.at<double>(1, 0) = point[1];
	pointW.at<double>(2, 0) = point[2];

	pointC = R * pointW + t;

	return pointC;
}

cv::Vec3d PnPSolver::getPointWorldCoords(cv::Vec3d point) const
{
	cv::Mat pointW = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat pointC = cv::Mat::zeros(3, 1, CV_64F);

	pointC.at<double>(0, 0) = point[0];
	pointC.at<double>(1, 0) = point[1];
	pointC.at<double>(2, 0) = point[2];

	pointW = R_inv * (pointC - t);

	return cv::Vec3d(pointW);
}

glm::vec3 PnPSolver::worldToOpenGLCoords(cv::Vec3d point) const
{
		return -glm::vec3(point[0], point[1], point[2]);
}

bool PnPSolver::wasSolvedLastFrame() const
{
	return solvedLastFrame;
}