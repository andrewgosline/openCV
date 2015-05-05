//#define USE_SENSOR
#define INCH_TO_CM 2.54
#define CAMERA_IDX 0  //On the Acer computer, the USB image capture is on 1, webcame on 0.

#define ENDOSCOPE_OFFSET_X 0 // -1.0  //offset to X in inches
#define ENDOSCOPE_OFFSET_Y 0 //-1.5  //offset to Y in inches
#define ENDOSCOPE_OFFSET_Z 0.8/INCH_TO_CM //0.3   //offset to Z in inches
#define TRACHEA_TARGET_OFFSET 2.7/INCH_TO_CM  //offsett to trachea in inches
#define ENDOSCOPE_VIEWING_ANGLE_DEFAULT 40.0 //cone angle of endoscope view default

#define IMAGE_CENTER_X 720/2 -40	//X - this defines where in the image the HeadsUpDisplay is centered
#define IMAGE_CENTER_Y 480/2	//Y - this defines where in the image the HeadsUpDisplay is centered
#define ENDOSCOPE_IMAGE_RADIUS 270			// Defines the radius of the HeadsUpDisplay in pixels.  

#define STRAIGHT_IDX		0
#define STRAIGHT_UP_IDX		1
#define STRAIGHT_DOWN_IDX	2
#define STRAIGHT_LEFT_IDX	3
#define STRAIGHT_RIGHT_IDX	4
#define UP_LEFT_IDX			5
#define UP_RIGHT_IDX		6
#define UP_IDX				7
#define DOWN_LEFT_IDX		8
#define DOWN_RIGHT_IDX		9
#define DOWN_IDX			10
#define LEFT_IDX			11
#define RIGHT_IDX			12

#define NUM_HEADSUP_COORDS	13

#define PI					3.141

class CSystem
{
public:
	SYSTEM_CONFIGURATION	m_config;
}; 

class CSensor
{
public:
	SENSOR_CONFIGURATION	m_config;
};

class CXmtr
{
public:
	TRANSMITTER_CONFIGURATION	m_config;
};

