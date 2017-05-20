#include "LevenbergMarquardt.h"
#include "mathTools.h"
#include "constants.h"


LevenbergMarquardt::LevenbergMarquardt()
{
	for (int i = 0; i < 4; i++) {
		pw[i] = cv::Mat::zeros(3, 1, CV_64F);
		u_n[i] = cv::Mat::zeros(3, 1, CV_64F);

		u_n[i].at<double>(2, 0) = 1.;
	}

	R = cv::Mat::zeros(3, 3, CV_64F);
	Rrod = cv::Mat::zeros(3, 1, CV_64F);
	t = cv::Mat::zeros(3, 1, CV_64F);
	Beta = cv::Mat::zeros(6, 1, CV_64F);
	Beta_temp = cv::Mat::zeros(6, 1, CV_64F);
	Err = cv::Mat::zeros(8, 1, CV_64F);
	J = cv::Mat::zeros(8, 6, CV_64F);
	Jrod = cv::Mat::zeros(3, 9, CV_64F);
	JtJdamp = cv::Mat::zeros(6, 6, CV_64F);
	Dir = cv::Mat::zeros(6, 1, CV_64F);
}

LevenbergMarquardt::~LevenbergMarquardt()
{
}

void LevenbergMarquardt::setPW(const std::vector<cv::Vec3d> & pw_init)
{
	for (int i = 0; i < 4; i++) {
		pw[i].at<double>(0, 0) = pw_init[i][0];
		pw[i].at<double>(1, 0) = pw_init[i][1];
		pw[i].at<double>(2, 0) = pw_init[i][2];
	}
}

void LevenbergMarquardt::init(const std::vector<cv::Vec2d> & u_n_init, cv::Mat & R_init, const cv::Mat & t_init)
{
	for (int i = 0; i < 4; i++) {
		u_n[i].at<double>(0, 0) = u_n_init[i][0];
		u_n[i].at<double>(1, 0) = u_n_init[i][1];
	}

	cv::Rodrigues(R_init, Rrod);
	for (int i = 0; i < 3; i++) {
		Beta.at<double>(i, 0) = Rrod.at<double>(i, 0);
		Beta.at<double>(i + 3, 0) = t_init.at<double>(i, 0);
	}
}

void LevenbergMarquardt::optimize()
{
	int iter = 0;
	double reprojError_prec = getMeanReprojectionError(Beta);
	double reprojError;
	double lambda = GET(LAMBDA_INIT);

	computeTransformation(Beta, true);
	computeJ();
	computeErr();

	while (iter < GET(MAX_ITER)) {
		iter++;

		JtJdamp = J.t() * J;
		JtJdamp.diag() *= 1. + lambda;
		Dir = (JtJdamp).inv(cv::DECOMP_SVD) * J.t() * Err;
		Beta_temp = Beta - Dir;
		reprojError = getMeanReprojectionError(Beta_temp);

		if (reprojError < reprojError_prec) {
			Beta = Beta_temp.clone();
			reprojError_prec = reprojError;
			lambda /= GET(LAMBDA_DOWN);
			computeTransformation(Beta, true);
			computeJ();
			computeErr();
		}
		else {
			lambda *= GET(LAMBDA_UP);
		}
	}
}

void LevenbergMarquardt::getResult(cv::Mat & R_res, cv::Mat & t_res)
{
	computeTransformation(Beta, false);
	R_res = R;
	t_res = t;
}

void LevenbergMarquardt::computeTransformation(const cv::Mat & B, bool getJacobian)
{
	for (int i = 0; i < 3; i++) {
		Rrod.at<double>(i, 0) = B.at<double>(i, 0);
		t.at<double>(i, 0) = B.at<double>(i + 3, 0);
	}

	if (getJacobian) {
		cv::Rodrigues(Rrod, R, Jrod);
	}
	else {
		cv::Rodrigues(Rrod, R);
	}
}

void LevenbergMarquardt::computeErr()
{
	cv::Mat pw_c;

	for (int pidx = 0; pidx < 4; pidx++) {
		pw_c = R * pw[pidx] + t;
		
		Err.at<double>(2 * pidx,     0) = u_n[pidx].at<double>(0, 0) * pw_c.at<double>(2, 0) - pw_c.at<double>(0, 0);
		Err.at<double>(2 * pidx + 1, 0) = u_n[pidx].at<double>(1, 0) * pw_c.at<double>(2, 0) - pw_c.at<double>(1, 0);
	}
}

void LevenbergMarquardt::computeJ()
{
	double temp[3];

	for (int pidx = 0; pidx < 4; pidx++) {
		// Derivative of Rrod
		for (int j = 0; j < 3; j++) {
			temp[0] = 0.;
			temp[1] = 0.;
			temp[2] = 0.;
			for (int k = 0; k < 3; k++) {
				temp[0] += Jrod.at<double>(j, k)     * pw[pidx].at<double>(k, 0);
				temp[1] += Jrod.at<double>(j, 3 + k) * pw[pidx].at<double>(k, 0);
				temp[2] += Jrod.at<double>(j, 6 + k) * pw[pidx].at<double>(k, 0);
			}

			J.at<double>(2 * pidx,     j) = u_n[pidx].at<double>(0, 0) * temp[2] - temp[0];
			J.at<double>(2 * pidx + 1, j) = u_n[pidx].at<double>(1, 0) * temp[2] - temp[1];
		}

		// Derivative of t
		J.at<double>(2 * pidx,     3) = -1.;
		J.at<double>(2 * pidx + 1, 4) = -1.;
		J.at<double>(2 * pidx,     5) = u_n[pidx].at<double>(0, 0);
		J.at<double>(2 * pidx + 1, 5) = u_n[pidx].at<double>(1, 0);
	}
}

double LevenbergMarquardt::getMeanReprojectionError(const cv::Mat & B)
{
	double error = 0.;
	cv::Mat pw_c;
	computeTransformation(B, false);

	for (int pidx = 0; pidx < 4; pidx++) {
		pw_c = R * pw[pidx] + t;
		pw_c /= pw_c.at<double>(2, 0);

		error += MathTools::diffSquareNorm(u_n[pidx], pw_c);
	}

	return error / 4.;
}
