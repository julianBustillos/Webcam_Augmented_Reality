#include "PnPSolver.h"

PnPSolver::PnPSolver(int width, int height) :
	uc(width / 2), vc(height / 2)
{
	pw[0] = cv::Vec3d (0.0,  0.0,  0.0);
	pw[1] = cv::Vec3d( 0.0, 10.0,  0.0);
	pw[2] = cv::Vec3d(10.0, 10.0,  0.0);
	pw[3] = cv::Vec3d(10.0,  0.0,  0.0);

	computeControlPoints();
	computeBarycentricCoords();
}

PnPSolver::~PnPSolver()
{
}

void PnPSolver::solve(std::vector<cv::Vec2i> corners)
{

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
	cw[1] = pca.eigenvectors.row(0);
	cw[1] += cw[0];
	cw[2] = pca.eigenvectors.row(1);
	cw[2] += cw[0];
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
}
