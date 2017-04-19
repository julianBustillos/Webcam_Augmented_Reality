#include "PnPSolver.h"
#include "constants.h"


PnPSolver::PnPSolver(int width, int height)
{
	uc[0] = height / 2;
	uc[1] = width / 2
		;
	pw[0] = cv::Vec3d (0.0,  0.0,  0.0);
	pw[1] = cv::Vec3d( 0.0, 10.0,  0.0);
	pw[2] = cv::Vec3d(10.0, 10.0,  0.0);
	pw[3] = cv::Vec3d(10.0,  0.0,  0.0);

	M = cv::Mat::zeros(8, 9, CV_64F);
	W = cv::Mat::zeros(8, 1, CV_64F);
	U = cv::Mat::zeros(8, 8, CV_64F);
	Vt = cv::Mat::zeros(9, 9, CV_64F);
	v = cv::Mat::zeros(9, 1, CV_64F);

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
	}

	fillM();
	computeMNullSpace();
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
		P.at<double>(3, j) = 1.0f;
	}

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {
			C.at<double>(i, j) = cw[j][i];
		}
		C.at<double>(3, j) = 1.0f;
	}

	cv::SVD::compute(C, W, U, Vt, cv::SVD::FULL_UV);

	for (int i = 0; i < 3; i++) {
		S_plus.at<double>(i, i) = 1.0f / W.at<double>(i);
	}

	alpha = Vt.t() * S_plus * U.t() * P;

	std::cout << C * alpha << std::endl;
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
	cv::SVD::compute(M, W, U, Vt, cv::SVD::FULL_UV);
	v = Vt.t().col(8);
	std::cout << v << std::endl;
	std::cout << M * v << std::endl << std::endl;
	
	cv::eigen(M.t() * M, eigenvalues, eigenvectors);
	std::cout << eigenvectors.row(8).t() << std::endl;
	std::cout << M * eigenvectors.row(8).t() << std::endl << std::endl;
}