#pragma once

#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"


void kalman_init(float&, float&, float&, cv::KalmanFilter&);
cv::Mat_<float> kalman(float, float, float, cv::KalmanFilter&);
void kalman_init1D(double&, cv::KalmanFilter&);
cv::Mat_<float> kalman1D(double, cv::KalmanFilter&);