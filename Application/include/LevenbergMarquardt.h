#pragma once

#include <opencv2/opencv.hpp>


class LevenbergMarquardt {
public:
	LevenbergMarquardt();
	~LevenbergMarquardt();
	void setPW(const std::vector<cv::Vec3d> & pw_init);
	void init(const std::vector<cv::Vec2d> & u_n_init, cv::Mat & R_init, const cv::Mat & t_init);
	void optimize();
	void getResult(cv::Mat & R_res, cv::Mat & t_res);

private:
	void computeTransformation(const cv::Mat & B, bool getJacobian);
	void computeErr();
	void computeJ();
	double getMeanReprojectionError(const cv::Mat & B);

	//DATA
	cv::Mat pw[4];
	cv::Mat u_n[4];
	cv::Mat R;
	cv::Mat Rrod;
	cv::Mat t;
	cv::Mat Beta;
	cv::Mat Beta_temp;
	cv::Mat Err;
	cv::Mat J;
	cv::Mat Jrod;
	cv::Mat JtJdamp;
	cv::Mat Dir;
};