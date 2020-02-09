#pragma once
#include "Kalman.h"
#include <Kinect.h>
#include <gl/glew.h>
#include <GL/freeglut.h>
#include <SFML/Audio.hpp>
#include <chrono>


/************************************************
 *  Class representing one instrument area
 ***********************************************/
class instrument {

	public:

		CameraSpacePoint centerPoint;		// Center point of the instrument collision area
		sf::SoundBuffer buf;
		double height = 0.1;
		double width = 0.45;
		double length = 4;

		boolean recentlyPlayedR = 0;		// Boolean to be set after playing, and reset once hand leaves instrument area
		boolean recentlyPlayedL = 0;


		/************************************************
		 *  Initialization with 0 values that are later to be set during calibration
		 ***********************************************/
		instrument() : centerPoint({ 0,0,0 }) {}

		/************************************************
		 *  Unused - initializing with specific coordinates
		 ***********************************************/
		instrument(CameraSpacePoint center) : centerPoint(center) {}

		/************************************************
		 *  Hand collision detection for given instrument area
		 ***********************************************/
		void detectCollision(sf::Sound&, sf::Sound&, sf::SoundBuffer&);

};


/************************************************
 *  Class specifically made for an instance of kick part in the drum
 ***********************************************/
class kick {

	public:

		sf::SoundBuffer buf;
		CameraSpacePoint kneeRest;		// Rest coordinate of the knee
		boolean recentlyPlayed = 1;		// Boolean to be set after playing, and reset once knee leaves kick drum part area
		
		/************************************************
		 * initializing with 0 values that are later to be set during calibration
		 ***********************************************/
		kick() : kneeRest({ 0,0,0 }) {}

		/************************************************
		 *  Collision detection for kick drum part
		 ***********************************************/
		void detectCollision();

};


/************************************************
 *  Kinect initializer
 ***********************************************/
bool initKinect();

/************************************************
 *  Function responsible for acquiring depth data from the sensor
 ***********************************************/
void getDepthData(IMultiSourceFrame*, GLubyte*);

/************************************************
 * Function responsible for acquiring color data from the sensor
 ***********************************************/
void getRgbData(IMultiSourceFrame*, GLubyte*);

/************************************************
 * Function performing body data acqusition into the global joints variable
 ***********************************************/
void getBodyData(IMultiSourceFrame*);

/************************************************
 * Composes all data together into kinect point cloud buffer to display
 ***********************************************/
void getKinectData();

/************************************************
 * Function drawing simple lines representing arms and knee position to reach in order to play kick sound
 ***********************************************/
void drawInstruments();

/************************************************
 * Function calls all the data acqusition functions and performs drawing of the arms skeleton
 ***********************************************/
void drawKinectData();

/************************************************
 * Speed of the arms measurement
 ***********************************************/
void measureSpeed(double&, double&);