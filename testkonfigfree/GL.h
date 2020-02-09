#pragma once
#include "KinectUtil.h"



/************************************************
 * GL initalization
 ***********************************************/
void initializeGL();

/************************************************
 * Function entering GL main loop
 ***********************************************/
void execute();

/************************************************
 * Calibration function,
 * awaits 5 seconds after beginning body tracking and sets instruments area accordingly to the spine coordinate
 ***********************************************/
void setInstrumentAreas();

/************************************************
 * Function being called every iteration of the GL main loop, sort of treated like update() function that somehow refused to work
 ***********************************************/
void draw();

/************************************************
 * Window initlization
 ***********************************************/
bool init(int argc, char* argv[]);

/************************************************
 * Rotation of the kinect point cloud around the middle point of reference system
 ***********************************************/
void rotateCamera();