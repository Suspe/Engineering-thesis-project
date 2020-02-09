#include "GL.h"

extern bool instrumentSet;

extern const int width;
extern const int height;

extern double SpeedR, SpeedL;
extern cv::Mat_<float> filteredL;
extern cv::Mat_<float> filteredR;

extern cv::KalmanFilter FilterPosL;
extern cv::KalmanFilter FilterPosR;
extern cv::KalmanFilter FilterSpeedL;
extern cv::KalmanFilter FilterSpeedR;

extern cv::Mat_<float> filteredSpeedR, filteredSpeedL;

extern GLuint vboId;
extern GLuint cboId;

extern sf::Sound foot;
extern sf::Sound left;					
extern sf::Sound right;

extern sf::SoundBuffer rimBuffer;			
extern sf::SoundBuffer ophatBuffer;
extern sf::SoundBuffer clhatBuffer;
extern sf::SoundBuffer kickBuffer;

extern BOOLEAN tracked;							// Whether we see a body
extern Joint joints[];				// List of joints in the tracked body

extern instrument rim, ophat, clhat;
extern kick kick;

// Kinect Variables
//IKinectSensor* sensor;             // Kinect sensor
//IMultiSourceFrameReader* reader;   // Kinect data source
//ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates


// Intermediate Buffers
//unsigned char rgbimage[colorwidth * colorheight * 4];    // Stores RGB color image
//ColorSpacePoint depth2rgb[width * height];               // Maps depth pixels to rgb pixels
//CameraSpacePoint depth2xyz[width * height];				 // Maps depth pixels to 3d coordinates


/************************************************
 * GL initalization
 ***********************************************/
void initializeGL() {

	// OpenGL setup
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0f);

	// Set up array buffers
	const int dataSize = width * height * 3 * 4;
	glGenBuffers(1, &vboId);
	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);
	glGenBuffers(1, &cboId);
	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);

	// Camera setup
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, width / (GLdouble)height, 0.1, 1000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);

}

/************************************************
 * Function entering GL main loop
 ***********************************************/
void execute() {
	glutMainLoop();
}

/************************************************
 * Calibration function,
 * awaits 5 seconds after beginning body tracking and sets instruments area accordingly to the spine coordinate
 ***********************************************/
void setInstrumentAreas() {

	if (tracked) {
		static std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		static std::chrono::high_resolution_clock::time_point current;
		current = std::chrono::high_resolution_clock::now();

		auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current - start).count();
		if (elapsed >= 5) {

			kick.kneeRest.X = joints[JointType_KneeRight].Position.X;
			kick.kneeRest.Y = joints[JointType_KneeRight].Position.Y;
			kick.kneeRest.Z = joints[JointType_KneeRight].Position.Z;

			rim.centerPoint.X = joints[JointType_SpineShoulder].Position.X + 0.3;
			rim.centerPoint.Y = joints[JointType_SpineShoulder].Position.Y - 0.45;
			rim.centerPoint.Z = joints[JointType_SpineShoulder].Position.Z - 0.2;

			clhat.centerPoint.X = joints[JointType_SpineShoulder].Position.X - 0.3;
			clhat.centerPoint.Y = joints[JointType_SpineShoulder].Position.Y - 0.45;
			clhat.centerPoint.Z = joints[JointType_SpineShoulder].Position.Z - 0.2;

			ophat.centerPoint.X = joints[JointType_SpineShoulder].Position.X + 0.67;
			ophat.centerPoint.Y = joints[JointType_SpineShoulder].Position.Y;
			ophat.centerPoint.Z = joints[JointType_SpineShoulder].Position.Z - 0.2;
			instrumentSet = 1;
		}
	}
}

/************************************************
 * Function being called every iteration of the GL main loop, sort of treated like update() function that somehow refused to work
 ***********************************************/
void draw() {
	drawKinectData();

	if (!instrumentSet)
		setInstrumentAreas();
	drawInstruments();
	measureSpeed(SpeedR, SpeedL);

	filteredL = kalman(joints[JointType_HandLeft].Position.X, joints[JointType_HandLeft].Position.Y, joints[JointType_HandLeft].Position.Z, FilterPosL);
	filteredR = kalman(joints[JointType_HandRight].Position.X, joints[JointType_HandRight].Position.Y, joints[JointType_HandRight].Position.Z, FilterPosR);
	filteredSpeedL = kalman1D(SpeedL, FilterSpeedL);
	filteredSpeedR = kalman1D(SpeedR, FilterSpeedR);

	rim.detectCollision(left, right, rimBuffer);
	ophat.detectCollision(left, right, ophatBuffer);
	clhat.detectCollision(left, right, clhatBuffer);
	kick.detectCollision();

	glutSwapBuffers();
}

/************************************************
 * Window initlization
 ***********************************************/
bool init(int argc, char* argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(width, height);
	glutCreateWindow("Virtual drums");
	glutDisplayFunc(draw);
	glutIdleFunc(draw);
	glewInit();
	return true;
}

/************************************************
 * Rotation of the kinect point cloud around the middle point of reference system
 ***********************************************/
void rotateCamera() {
	static double angle = 0.;
	static double radius = 3.;
	double x = radius * sin(angle);
	double z = radius * (1 - cos(angle)) - radius / 2;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(x, 0, z, 0, 0, radius / 2, 0, 1, 0);
	angle += 0.001;
}