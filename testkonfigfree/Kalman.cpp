#include "Kalman.h"

using namespace cv;

// Kalman filter Variables
KalmanFilter FilterPosL(6, 3, 0);
KalmanFilter FilterPosR(6, 3, 0);
KalmanFilter FilterSpeedL(2, 1, 0);
KalmanFilter FilterSpeedR(2, 1, 0);

void kalman_init(float &x, float &y, float &z, KalmanFilter &Filter) {
	
	Filter.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,    0, 1, 0, 0, 1, 0,    0, 0, 1, 0, 0, 1,    0, 0, 0, 1, 0, 0,   0, 0, 0, 0, 1, 0,    0, 0, 0, 0, 0, 1);

	Filter.statePre.at<float>(0) = x;        // Position x
	Filter.statePre.at<float>(1) = y;        // Position y 
	Filter.statePre.at<float>(2) = z;        // Position z
	Filter.statePre.at<float>(3) = 0;        // Speed along x axis
	Filter.statePre.at<float>(4) = 0;		 // Speed along y axis
	Filter.statePre.at<float>(5) = 0;		 // Speed along z axis

	setIdentity(Filter.measurementMatrix);
	setIdentity(Filter.processNoiseCov, Scalar::all(1e-5));        // Covariance Q 
	setIdentity(Filter.measurementNoiseCov, Scalar::all(1e-3));    // Covariance R - influences K - Kalman's amplification
	setIdentity(Filter.errorCovPost, Scalar::all(.1));
}

void kalman_init1D(double& x, KalmanFilter& Filter) {

	Filter.transitionMatrix = (Mat_<float>(2,2) << 1, 1, 0, 1);

	Filter.statePre.at<float>(0) = x;        // Position x
	Filter.statePre.at<float>(1) = 0;        // Speed along x axis

	setIdentity(Filter.measurementMatrix);
	setIdentity(Filter.processNoiseCov, Scalar::all(1e-5));        // Covariance Q 
	setIdentity(Filter.measurementNoiseCov, Scalar::all(1e-3));    // Covariance R - influences K - Kalman's amplification
	setIdentity(Filter.errorCovPost, Scalar::all(.1));
}

Mat_<float> kalman1D(double x, KalmanFilter& Filter) {

	Mat_<float> measured(1, 1);
	measured(0) = x;

	Filter.predict();

	Mat estimated = Filter.correct(measured);

	return estimated;
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
