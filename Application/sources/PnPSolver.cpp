#include "PnPSolver.h"
#include "constants.h"
#include <chrono>
#include "mathTools.h"


PnPSolver::PnPSolver(int width, int height)
{
	uc[0] = height / 2;
	uc[1] = width / 2
		;
	pw[0] = cv::Vec3d (0.,  0.,  0.);
	pw[1] = cv::Vec3d(-10.,  0.,  0.);
	pw[2] = cv::Vec3d(-10.,  0., -10.);
	pw[3] = cv::Vec3d( 0.,  0., -10.);

	M = cv::Mat::zeros(8, 9, CV_64F);
	vDist = cv::Mat::zeros(3, 1, CV_64F);
	cwDist = cv::Mat::zeros(3, 1, CV_64F);
	R = cv::Mat::zeros(3, 3, CV_64F);
	R_inv = cv::Mat::zeros(3, 3, CV_64F);
	t = cv::Mat::zeros(3, 1, CV_64F);

	computeFocalLength();
	computeControlPoints();
	computeBarycentricCoords();
}

PnPSolver::~PnPSolver()
{
}

void PnPSolver::solve(std::vector<cv::Vec2i> corners)
{
	for (int idx = 0; idx < 4; idx++) {
			u[idx] = corners[idx];
			std::cout << corners[idx] << std::endl;
	}

	fillM();
	computeMNullSpace();
	computeBeta();
	computePC();
	estimateTransformation();
}

glm::vec3 PnPSolver::getCameraPosition() const
{
	cv::Vec3d cameraPositionC = cv::Vec3d(0.0f, 0.0f, 0.0f);
	cv::Vec3d cameraPositionW = getPointWorldCoords(cameraPositionC);

	std::cout << "POSITION : " << cameraPositionW << std::endl;
	return worldToOpenGLCoords(cameraPositionW);
}

glm::vec3 PnPSolver::getCameraFront(const glm::vec3 & cameraPositionW) const
{
	cv::Vec3d cameraFrontC = cv::Vec3d(0.0f, 0.0f, 1.0f);
	cv::Vec3d cameraFrontW = getPointWorldCoords(cameraFrontC);
	glm::vec3 cameraFrontRes = glm::normalize(worldToOpenGLCoords(cameraFrontW) - cameraPositionW);

	std::cout << "FRONT    : " << cv::Vec3d(cameraFrontRes.x, cameraFrontRes.y, cameraFrontRes.z) << std::endl;
	return cameraFrontRes;
}

glm::vec3 PnPSolver::getCameraUp(const glm::vec3 & cameraPositionW) const
{
	cv::Vec3d cameraUpC = cv::Vec3d(-1.0f, 0.0f, 0.0f);
	cv::Vec3d cameraUpW = getPointWorldCoords(cameraUpC);
	glm::vec3 cameraUpRes = glm::normalize(worldToOpenGLCoords(cameraUpW) - cameraPositionW);

	std::cout << "UP       : " << cv::Vec3d(cameraUpRes.x, cameraUpRes.y, cameraUpRes.z) << std::endl;
	return cameraUpRes;
}

void PnPSolver::computeFocalLength()
{
	f[0] = f[1] = sqrt(uc[0] * uc[0] + uc[1] * uc[1]) / tan(GET(DFOV) * M_PI / 360);
}

void PnPSolver::computeControlPoints()
{
	cv::Mat data = cv::Mat::zeros(4, 3, CV_64F);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			data.at<double>(i, j) = pw[i][j];
		}
	}

	cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

	cw[0] = pca.mean;
	for (int idx = 1; idx < 3; idx++) {
		cw[idx] = pca.eigenvectors.row(idx - 1);
		cw[idx] *= sqrt(pca.eigenvalues.at<double>(idx - 1));
		cw[idx] += cw[0];
	}
}

void PnPSolver::computeBarycentricCoords()
{
	alpha = cv::Mat::zeros(3, 4, CV_64F);
	cv::Mat P = cv::Mat::zeros(4, 4, CV_64F);
	cv::Mat C = cv::Mat::zeros(4, 3, CV_64F);
	cv::Mat S_plus = cv::Mat::zeros(3, 4, CV_64F);
	cv::Mat W;
	cv::Mat U;
	cv::Mat Vt;

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 3; i++) {
			P.at<double>(i, j) = pw[j][i];
		}
		P.at<double>(3, j) = 1.;
	}

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {
			C.at<double>(i, j) = cw[j][i];
		}
		C.at<double>(3, j) = 1.;
	}

	cv::SVD::compute(C, W, U, Vt, cv::SVD::FULL_UV);

	for (int i = 0; i < 3; i++) {
		S_plus.at<double>(i, i) = 1. / W.at<double>(i);
	}

	alpha = Vt.t() * S_plus * U.t() * P;
}

void PnPSolver::fillM()
{
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			M.at<double>(i * 2, j * 3) = alpha.at<double>(j, i) * f[0];
			M.at<double>(i * 2, j * 3 + 2) = alpha.at<double>(j, i) * (uc[0] - u[i][0]);
			M.at<double>(i * 2 + 1, j * 3 + 1) = alpha.at<double>(j, i) * f[1];
			M.at<double>(i * 2 + 1, j * 3 + 2) = alpha.at<double>(j, i) * (uc[1] - u[i][1]);
		}
	}
}

void PnPSolver::computeMNullSpace()
{
	cv::Mat W;
	cv::Mat U;
	cv::Mat Vt;
	cv::Mat vData;

	cv::SVD::compute(M, W, U, Vt, cv::SVD::FULL_UV);

	vData = Vt.t().col(8);
	v[0] = cv::Vec3d(vData.at<double>(0, 0), vData.at<double>(1, 0), vData.at<double>(2, 0));
	v[1] = cv::Vec3d(vData.at<double>(3, 0), vData.at<double>(4, 0), vData.at<double>(5, 0));
	v[2] = cv::Vec3d(vData.at<double>(6, 0), vData.at<double>(7, 0), vData.at<double>(8, 0));
}

void PnPSolver::computeBeta()
{
	cv::Mat W;
	cv::Mat U;
	cv::Mat Vt;
	cv::Mat S_plus = cv::Mat::zeros(1, 3, CV_64F);
	cv::Mat betaMat;

	cwDist.at<double>(0, 0) = MathTools::diffSquareNorm(cw[0], cw[1]);
	cwDist.at<double>(1, 0) = MathTools::diffSquareNorm(cw[0], cw[2]);
	cwDist.at<double>(2, 0) = MathTools::diffSquareNorm(cw[1], cw[2]);

	vDist.at<double>(0, 0) = MathTools::diffSquareNorm(v[0], v[1]);
	vDist.at<double>(1, 0) = MathTools::diffSquareNorm(v[0], v[2]);
	vDist.at<double>(2, 0) = MathTools::diffSquareNorm(v[1], v[2]);

	cv::SVD::compute(vDist, W, U, Vt, cv::SVD::FULL_UV);

	S_plus.at<double>(0, 0) = 1. / W.at<double>(0, 0);
	betaMat = Vt.t() * S_plus * U.t() * cwDist;
	beta = betaMat.at<double>(0, 0);
	beta = (beta > 0.) ? sqrt(beta) : sqrt(-beta);
}

void PnPSolver::computePC()
{
	for (int i = 0; i < 4; i++) {
		pc[i] = cv::Vec3d();
		for (int j = 0; j < 3; j++) {
			pc[i] += alpha.at<double>(j, i) * beta * v[j];
		}
	}
}

void PnPSolver::estimateTransformation()
{
	cv::Mat mu = cv::Mat(3, 2, CV_64F);
	double sigma = 0.;
	cv::Mat X = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat Y = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat Sigma = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat S = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat U;
	cv::Mat D;
	cv::Mat Vt;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			mu.at<double>(j, 0) += pw[i][j];
			mu.at<double>(j, 1) += pc[i][j];
		}
	}
	mu /= 4;

	for (int i = 0; i < 4; i++) {
		sigma += MathTools::diffSquareNorm(pw[i], cv::Vec3d(mu.col(0)));
	}
	sigma /= 4;

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 3; i++) {
			X.at<double>(i, 0) = pw[j][i] - mu.at<double>(i, 0);
			Y.at<double>(i, 0) = pc[j][i] - mu.at<double>(i, 1);
		}
		Sigma += Y * X.t();
	}
	Sigma /= 4;

	cv::SVD::compute(Sigma, D, U, Vt, cv::SVD::FULL_UV);

	if (MathTools::det3x3(U) * MathTools::det3x3(Vt) < 0) {
		S.at<double>(2, 2) = -1.;
	}

	R = U * S * Vt;
	R_inv = R.inv();
	t = mu.col(1) - R * mu.col(0);
	c = MathTools::trace3x1(S * D) / sigma;
	
}

double PnPSolver::getMeanReprojectionError() const
{
	double error = 0.;

	for (int i = 0; i < 4; i++) {
		error += MathTools::diffSquareNorm(pc[i], getPointCameraCoords(pw[i]));
	}

	return error / 4;
}

cv::Vec3d PnPSolver::getPointCameraCoords(cv::Vec3d point) const
{
	cv::Mat pointW = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat pointC = cv::Mat::zeros(3, 1, CV_64F);

	pointW.at<double>(0, 0) = point[0];
	pointW.at<double>(1, 0) = point[1];
	pointW.at<double>(2, 0) = point[2];

	pointC = c * R * pointW + t;

	return cv::Vec3d(pointC);
}

cv::Vec3d PnPSolver::getPointWorldCoords(cv::Vec3d point) const
{
	cv::Mat pointW = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat pointC = cv::Mat::zeros(3, 1, CV_64F);

	pointC.at<double>(0, 0) = point[0];
	pointC.at<double>(1, 0) = point[1];
	pointC.at<double>(2, 0) = point[2];

	pointW = R_inv * (pointC - t) / c;

	return cv::Vec3d(pointW);
}

glm::vec3 PnPSolver::worldToOpenGLCoords(cv::Vec3d point) const
{
	return -glm::vec3(point[0], point[1], point[2]);
}
