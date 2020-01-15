#include "Kalman.h"
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;

void kalman_init(float &x, float &y, float &z, KalmanFilter &Filter) {
	
	Filter.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,    0, 1, 0, 0, 1, 0,    0, 0, 1, 0, 0, 1,    0, 0, 0, 1, 0, 0,   0, 0, 0, 0, 1, 0,    0, 0, 0, 0, 0, 1);

	Filter.statePre.at<float>(0) = x;        //Po³o¿enie x 
	Filter.statePre.at<float>(1) = y;        //Po³o¿enie y 
	Filter.statePre.at<float>(2) = z;        //Po³o¿enie z 
	Filter.statePre.at<float>(3) = 0;        //Prêdkoœæ x 
	Filter.statePre.at<float>(4) = 0;		 //Prêdkoœæ y
	Filter.statePre.at<float>(5) = 0;		 //Prêdkoœæ z

	setIdentity(Filter.measurementMatrix);
	setIdentity(Filter.processNoiseCov, Scalar::all(1e-4));        //Kowariancja Q 
	setIdentity(Filter.measurementNoiseCov, Scalar::all(1e-1));    //Kowariancja R 
	setIdentity(Filter.errorCovPost, Scalar::all(.1));
}

Mat_<float> kalman(float x, float y, float z, KalmanFilter &Filter) {

	Mat_<float> measured(3, 1);
	measured(0) = x;
	measured(1) = y;
	measured(2) = z;

	Filter.predict();

	Mat estimated = Filter.correct(measured);

	return estimated;
}
