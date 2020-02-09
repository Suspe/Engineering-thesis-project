#include "KinectUtil.h"

bool instrumentSet = 0;			// variable to be set once calibration is carried out

// Drum Variables 
double SpeedR, SpeedL;							    // Instantaneous speed of hands
cv::Mat_<float> filteredSpeedR, filteredSpeedL;		// Variable used for storing filtered speed values

// Window parameters
extern const int width = 512;
extern const int height = 424;

// Pixels mapping parameters
const int colorwidth = 1920;
const int colorheight = 1080;

// Filtered joint data
cv::Mat_<float> filteredL(3, 1);				// Stores filtered speed of the hand values
cv::Mat_<float> filteredR(3, 1);

// SFML Variables
sf::Sound left;						// SFML audio object for sound playing
sf::Sound right;
sf::Sound foot;
sf::SoundBuffer rimBuffer;			// Buffers storing wav files for instrument objects
sf::SoundBuffer ophatBuffer;
sf::SoundBuffer clhatBuffer;
sf::SoundBuffer kickBuffer;

// Body tracking Variables
BOOLEAN tracked;							// Whether we see a body
Joint joints[JointType_Count];				// List of joints in the tracked body

// Kinect Variables
IKinectSensor* sensor;             // Kinect sensor
IMultiSourceFrameReader* reader;   // Kinect data source
ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates

// We'll be using buffer objects to store the kinect point cloud
GLuint vboId;
GLuint cboId;

// Intermediate Buffers
unsigned char rgbimage[colorwidth * colorheight * 4];    // Stores RGB color image
ColorSpacePoint depth2rgb[width * height];               // Maps depth pixels to rgb pixels
CameraSpacePoint depth2xyz[width * height];				 // Maps depth pixels to 3d coordinates

kick kick;
instrument rim, ophat, clhat;


/************************************************
 *  Hand collision detection for given instrument area
 ***********************************************/
void instrument::detectCollision(sf::Sound& left, sf::Sound& right, sf::SoundBuffer& buf) {
		if (tracked) {
			const CameraSpacePoint& LH = joints[JointType_HandLeft].Position;
			const CameraSpacePoint& RH = joints[JointType_HandRight].Position;

			if (this->recentlyPlayedL == 0) {
				if (SpeedL < 0)
					if ((filteredL(0) >= this->centerPoint.X - this->width / 2) && (filteredL(0) <= this->centerPoint.X + this->width / 2))
						if ((filteredL(1) <= this->centerPoint.Y + this->height / 2) && (filteredL(1) >= this->centerPoint.Y - this->height / 2))
							if ((filteredL(2) <= this->centerPoint.Z + this->length / 2) && (filteredL(2) >= this->centerPoint.Z - this->length / 2)) {
								left.setBuffer(buf);
								left.setVolume(abs(1000 * filteredSpeedL(0) * 33));
								left.setPitch(1);// +6 * abs(filteredSpeedL(0)));
								left.play();
								this->recentlyPlayedL = 1;
							}
			}
			else {
				if (filteredL(1) >= this->centerPoint.Y + 0.6 * this->height)
					this->recentlyPlayedL = 0;
			}

			if (this->recentlyPlayedR == 0) {
				if (SpeedR < 0)
					if ((filteredR(0) >= this->centerPoint.X - this->width / 2) && (filteredR(0) <= this->centerPoint.X + this->width / 2))
						if ((filteredR(1) <= this->centerPoint.Y + this->height / 2) && (filteredR(1) >= this->centerPoint.Y - this->height / 2))
							if ((filteredR(2) <= this->centerPoint.Z + this->length / 2) && (filteredR(2) >= this->centerPoint.Z - this->length / 2)) {
								right.setBuffer(buf);
								right.setVolume(abs(1000 * filteredSpeedR(0) * 33));
								right.setPitch(1);// +6 * abs(filteredSpeedR(0)));
								right.play();
								this->recentlyPlayedR = 1;
							}
			}
			else {
				if (filteredR(1) >= this->centerPoint.Y + 0.6 * this->height)
					this->recentlyPlayedR = 0;
			}
		}
}


/************************************************
 *  Collision detection for kick drum part
 ***********************************************/
void kick::detectCollision() {
		if (joints[JointType_KneeRight].Position.Y - this->kneeRest.Y >= 0.020)
			this->recentlyPlayed = 0;
		if (joints[JointType_KneeRight].Position.Y - this->kneeRest.Y <= 0.01 && this->recentlyPlayed == 0) {
			foot.play();
			this->recentlyPlayed = 1;
		}
}

/************************************************
 *  Kinect initializer
 ***********************************************/
bool initKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Body,
			&reader);
		return reader;
	}
	else {
		return false;
	}
}

/************************************************
 *  Function responsible for acquiring depth data from the sensor
 ***********************************************/
void getDepthData(IMultiSourceFrame* frame, GLubyte* dest) {
	IDepthFrame* depthframe;
	IDepthFrameReference* frameref = NULL;
	frame->get_DepthFrameReference(&frameref);
	frameref->AcquireFrame(&depthframe);
	if (frameref) frameref->Release();

	if (!depthframe) return;

	// Get data from frame
	unsigned int sz;
	unsigned short* buf;
	depthframe->AccessUnderlyingBuffer(&sz, &buf);

	// Write vertex coordinates
	mapper->MapDepthFrameToCameraSpace(sz, buf, width * height, depth2xyz);
	float* fdest = (float*)dest;
	for (int i = 0; i < sz; i++) {
		*fdest++ = depth2xyz[i].X;
		*fdest++ = depth2xyz[i].Y;
		*fdest++ = depth2xyz[i].Z;
	}

	// Fill in depth2rgb map
	mapper->MapDepthFrameToColorSpace(sz, buf, width * height, depth2rgb);
	if (depthframe) depthframe->Release();
}

/************************************************
 * Function responsible for acquiring color data from the sensor
 ***********************************************/
void getRgbData(IMultiSourceFrame* frame, GLubyte* dest) {
	IColorFrame* colorframe;
	IColorFrameReference* frameref = NULL;
	frame->get_ColorFrameReference(&frameref);
	frameref->AcquireFrame(&colorframe);
	if (frameref) frameref->Release();

	if (!colorframe) return;

	// Get data from frame
	colorframe->CopyConvertedFrameDataToArray(colorwidth * colorheight * 4, rgbimage, ColorImageFormat_Rgba);

	// Write color array for vertices
	ColorSpacePoint* p = depth2rgb;
	float* fdest = (float*)dest;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (p->X < 0 || p->Y < 0 || p->X > colorwidth || p->Y > colorheight) {
				*fdest++ = 0;
				*fdest++ = 0;
				*fdest++ = 0;
			}
			else {
				int idx = (int)p->X + colorwidth * (int)p->Y;
				*fdest++ = rgbimage[4 * idx + 0] / 255.;
				*fdest++ = rgbimage[4 * idx + 1] / 255.;
				*fdest++ = rgbimage[4 * idx + 2] / 255.;
			}
			// Don't copy alpha channel
			p++;
		}
	}

	if (colorframe) colorframe->Release();
}

/************************************************
 * Function performing body data acqusition into the global joints variable
 ***********************************************/
void getBodyData(IMultiSourceFrame* frame) {
	IBodyFrame* bodyframe;
	IBodyFrameReference* frameref = NULL;
	frame->get_BodyFrameReference(&frameref);
	frameref->AcquireFrame(&bodyframe);
	if (frameref) frameref->Release();

	if (!bodyframe) return;

	IBody* body[BODY_COUNT] = { 0 };
	bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
	for (int i = 0; i < BODY_COUNT; i++) {
		body[i]->get_IsTracked(&tracked);
		if (tracked) {
			body[i]->GetJoints(JointType_Count, joints);
			break;
		}
	}
	if (bodyframe) bodyframe->Release();
}

/************************************************
 * Composes all data together into kinect point cloud buffer to display
 ***********************************************/
void getKinectData() {
	IMultiSourceFrame* frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		GLubyte* ptr;
		glBindBuffer(GL_ARRAY_BUFFER, vboId);
		ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		if (ptr) {
			getDepthData(frame, ptr);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, cboId);
		ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		if (ptr) {
			getRgbData(frame, ptr);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		getBodyData(frame);
	}
	if (frame) frame->Release();
}

/************************************************
 * Function drawing simple lines representing arms and knee position to reach in order to play kick sound
 ***********************************************/
void drawInstruments() {
	glVertex3f(rim.centerPoint.X - rim.width / 2, rim.centerPoint.Y + rim.height / 2, rim.centerPoint.Z);
	glVertex3f(rim.centerPoint.X + rim.width / 2, rim.centerPoint.Y + rim.height / 2, rim.centerPoint.Z);

	glVertex3f(clhat.centerPoint.X - clhat.width / 2, clhat.centerPoint.Y + clhat.height / 2, clhat.centerPoint.Z);
	glVertex3f(clhat.centerPoint.X + clhat.width / 2, clhat.centerPoint.Y + clhat.height / 2, clhat.centerPoint.Z);

	glVertex3f(ophat.centerPoint.X - ophat.width / 2, ophat.centerPoint.Y + ophat.height / 2, ophat.centerPoint.Z);
	glVertex3f(ophat.centerPoint.X + ophat.width / 2, ophat.centerPoint.Y + ophat.height / 2, ophat.centerPoint.Z);

	glVertex3f(kick.kneeRest.X - 0.01, kick.kneeRest.Y + 0.022, kick.kneeRest.Z);
	glVertex3f(kick.kneeRest.X + 0.01, kick.kneeRest.Y + 0.022, kick.kneeRest.Z);
}

/************************************************
 * Function calls all the data acqusition functions and performs drawing of the arms skeleton
 ***********************************************/
void drawKinectData() {
	getKinectData();
	//rotateCamera();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, vboId);//
	glVertexPointer(3, GL_FLOAT, 0, NULL);

	glBindBuffer(GL_ARRAY_BUFFER, cboId);//
	glColorPointer(3, GL_FLOAT, 0, NULL);

	glPointSize(1.f);
	glDrawArrays(GL_POINTS, 0, width * height);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	if (tracked) {

		const CameraSpacePoint& lw = joints[JointType_WristLeft].Position;
		const CameraSpacePoint& le = joints[JointType_ElbowLeft].Position;;
		const CameraSpacePoint& ls = joints[JointType_ShoulderLeft].Position;
		const CameraSpacePoint& rw = joints[JointType_WristRight].Position;
		const CameraSpacePoint& re = joints[JointType_ElbowRight].Position;
		const CameraSpacePoint& rs = joints[JointType_ShoulderRight].Position;
		const CameraSpacePoint& rh = joints[JointType_HandRight].Position;
		const CameraSpacePoint& lh = joints[JointType_HandLeft].Position;
		const CameraSpacePoint& lht = joints[JointType_HandTipLeft].Position;
		const CameraSpacePoint& rht = joints[JointType_HandTipRight].Position;
		const CameraSpacePoint& lt = joints[JointType_ThumbLeft].Position;
		const CameraSpacePoint& rt = joints[JointType_ThumbRight].Position;
		const CameraSpacePoint& neck = joints[JointType_Neck].Position;
		const CameraSpacePoint& rk = joints[JointType_KneeRight].Position;

		Joint joint = joints[JointType_HandLeft];
		if (joint.TrackingState == TrackingState_Inferred)
			std::cout << "inferred" << std::endl;

		glLineWidth(5);
		glBegin(GL_LINES);
		glColor3f(1.f, 0.f, 0.f);

		glVertex3f(filteredL(0), filteredL(1), filteredL(2));
		glVertex3f(lw.X, lw.Y, lw.Z);

		glVertex3f(lw.X, lw.Y, lw.Z);
		glVertex3f(le.X, le.Y, le.Z);

		glVertex3f(le.X, le.Y, le.Z);
		glVertex3f(ls.X, ls.Y, ls.Z);

		glVertex3f(filteredR(0), filteredR(1), filteredR(2));
		glVertex3f(rw.X, rw.Y, rw.Z);

		glVertex3f(rw.X, rw.Y, rw.Z);
		glVertex3f(re.X, re.Y, re.Z);

		glVertex3f(re.X, re.Y, re.Z);
		glVertex3f(rs.X, rs.Y, rs.Z);

		glVertex3f(rk.X + 0.01, rk.Y, rk.Z);
		glVertex3f(rk.X - 0.01, rk.Y, rk.Z);

		//instrument area 
		drawInstruments();

		glEnd();
	}

	else {
		std::cout << "body untracked" << std::endl;
	}

}


/************************************************
 * Speed of the arms measurement
 ***********************************************/
void measureSpeed(double& SpeedR, double& SpeedL)
{
	static double prevR = filteredR(1);
	static double prevL = filteredL(1);
	static std::chrono::high_resolution_clock::time_point prevTime = std::chrono::high_resolution_clock::now();

	double LH = filteredL(1);
	double RH = filteredR(1);

	std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - prevTime).count();

	if (LH != prevL)
		if (elapsed != 0)
			SpeedL = (LH - prevL) / (elapsed);

	if (RH != prevR)
		if (elapsed != 0)
			SpeedR = (RH - prevR) / (elapsed);

	prevR = filteredR(1);
	prevL = filteredL(1);
	prevTime = std::chrono::high_resolution_clock::now();
}
