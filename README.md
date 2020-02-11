# Engineering thesis project
Building virtual drums using depth sensor.

The task is to build a virtual drums system. It's main functionality is anticipated to be playing sound in real time, depending on the movement of the hand/leg, like it takes place in case of psychical percussion. Main assumption is that Kinect 2.0 and it's software development kit shall be used. Expected result is a functioning prototype that implements instrument areas with which collision of the hand shall generate adequate sound depending on speed of the hand, and which area has been triggered. Playing should take place with no noticable delay. 

Concept posited in the project includes body tracking with the usage of depth sensor and it's native software development kit. Coordinates of the user's joints will be tracked and compared to positions of the instrument areas. If that collision detection returns positive, an adequate .wav sample containing proper percussion sound will be played using audio library. Volume parameter is adjusted depending on the measured hand speed in the moment of trigger. After a few seconds after running the program some callibration will be carried out in order to set proper drum areas coordinates in relation to user's body. Kalman's filter will be used to cut off the noise and sharp measurement deviations.

Used libraries: 
- OpenCV (Kalman's filter implementation from tracking module)
- SFML (audio module)
- Kinect for Windows SDK 2.0 (to get sensor running, body tracking algorithm)
- Freeglut (accessory visualization)

Project has been developed in Visual Studio Community.
Kinect 2.0 sensor is required for program to work for obvious reasons.


