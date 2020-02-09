#include <Windows.h>
#include <cmath>
#include <string>
#include <Ole2.h>
#include <cstdlib>
#include <iomanip>
#include <fstream>
#include "GL.h"

extern cv::KalmanFilter FilterPosL;
extern cv::KalmanFilter FilterPosR;
extern cv::KalmanFilter FilterSpeedL;
extern cv::KalmanFilter FilterSpeedR;

extern double SpeedR, SpeedL;
extern Joint joints[];
extern sf::Sound foot;

extern sf::SoundBuffer rimBuffer;
extern sf::SoundBuffer ophatBuffer;
extern sf::SoundBuffer clhatBuffer;
extern sf::SoundBuffer kickBuffer;

int main(int argc, char* argv[]) {
	if (!init(argc, argv)) return 1;
	if (!initKinect()) return 1;
	initializeGL();

	rimBuffer.loadFromFile("rim.wav");
	ophatBuffer.loadFromFile("ophat.wav");
	clhatBuffer.loadFromFile("clhat.wav");
	kickBuffer.loadFromFile("kick.wav");
	foot.setBuffer(kickBuffer);
	foot.setVolume(100);
	foot.setPitch(1);

	kalman_init(joints[JointType_HandLeft].Position.X, joints[JointType_HandLeft].Position.Y, joints[JointType_HandLeft].Position.Z, FilterPosL);
	kalman_init(joints[JointType_HandRight].Position.X, joints[JointType_HandRight].Position.Y, joints[JointType_HandRight].Position.Z, FilterPosR);
	kalman_init1D(SpeedL, FilterSpeedL);
	kalman_init1D(SpeedR, FilterSpeedR);

	// Main loop
	execute();
	return 0;
}