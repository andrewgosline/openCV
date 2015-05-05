#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <armadillo>
#include <math.h>
#include "SampleFunctions.cpp"
using namespace arma;
//////////////////////////////////////////////////////////////////////////////
//	Main program:
int main() {
	char			distText[255];
	char			displayToggleText[255];
	char			rollText[255];
	char			yzOffsetText[255];
	char			viewAngleText[255];
	char			tracheaOffsetText[255];
	static int		count = 0;
	static int		toggleDisplayState = 0;
	static int		toggleGuidanceState = 1;
	double			distToTarget = -1;
	double			angle = 0;
	double			thetaOffset = 0;
	double			viewingAngle = ENDOSCOPE_VIEWING_ANGLE_DEFAULT;
	double			tracheaOffset = TRACHEA_TARGET_OFFSET;
	double			yOffset= ENDOSCOPE_OFFSET_Y;
	double			zOffset= ENDOSCOPE_OFFSET_Z;
	int				keyPress=0;
	CvCapture*		capture = cvCaptureFromCAM(CAMERA_IDX); //cvCaptureFromCAM(CV_CAP_DSHOW);
	CvFont			font;
	double			hScale=0.5;
	double			vScale=0.5;
	int				lineWidth=2;
	IplImage*		frame;
	int				i = 0;
	mat				headsUpLocations(NUM_HEADSUP_COORDS,2); headsUpLocations.fill(0);
	vec				endoscopeOffsets(3); endoscopeOffsets(0) = ENDOSCOPE_OFFSET_X; endoscopeOffsets(1) = ENDOSCOPE_OFFSET_Y; endoscopeOffsets(2) = ENDOSCOPE_OFFSET_Z; 
	int				imageLocation = STRAIGHT_LEFT_IDX;
	CvScalar		displayTextColor; displayTextColor = CV_RGB(0,125,0);
	CvScalar		headsUpColor; headsUpColor = CV_RGB(255,0,0);
	CvScalar		defaultTextColor; defaultTextColor = CV_RGB(0,200,0);

	headsUpLocations = createHeadsUpCoords(IMAGE_CENTER_X, IMAGE_CENTER_Y, ENDOSCOPE_IMAGE_RADIUS, 1.1); // 12 and 10 are textbox offsets
	sprintf_s(displayToggleText, "Guidance ON");
	printf("Computer Aided Intubation Demo Starting\n");

#ifndef USE_SENSOR  // tell user that EM sensor is commented out
	printf(" Skipping EM Sensor because USE_SENSOR not defined\n");
#endif
#ifdef USE_SENSOR // comment out all the TrakSTAR variables and initialization
	//////////////////////////////////////////////////////////////////////////////
	// Initialize the ATC3DG driver and DLL
	CSystem			ATC3DG;
	CSensor			*pSensor;
	CXmtr			*pXmtr;
	int				errorCode;
	int				sensorID=0;
	short			id;
	int				records = 10;	
	DOUBLE_POSITION_MATRIX_RECORD sens0Target,  *pSens0Target = &sens0Target;
	DOUBLE_POSITION_MATRIX_RECORD sens1Scope,  *pSens1Scope = &sens1Scope;
	mat				sens0TargetTrans(4,4); sens0TargetTrans.fill(0);
	mat				sens1ScopeTrans(4,4); sens1ScopeTrans.fill(0);
	mat				sens0TargetTransExt(4,4); sens0TargetTransExt.fill(0);
	vec				targetInScopeFrame(4); targetInScopeFrame.fill(0);

	printf("Initializing ATC TrakSTAR system...\n");
	errorCode = InitializeBIRDSystem();
	if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
	//////////////////////////////////////////////////////////////////////////////
	// GET SYSTEM CONFIGURATION
	DATA_FORMAT_TYPE format = DOUBLE_POSITION_MATRIX;
	errorCode = SetSensorParameter(
	sensorID, // index number of target sensor
	DATA_FORMAT, // command parameter type
	&format, // address of data source buffer
	sizeof(format) // size of source buffer
	);
	if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
	sensorID = 1;
	errorCode = SetSensorParameter(sensorID, DATA_FORMAT, &format, sizeof(format) );
	if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
	// Get system configuration, user must provide an error handler
	errorCode = GetBIRDSystemConfiguration(&ATC3DG.m_config);
	if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
	//////////////////////////////////////////////////////////////////////////////
	// GET SENSOR CONFIGURATION
	pSensor = new CSensor[ATC3DG.m_config.numberSensors];
	for(i=0;i<ATC3DG.m_config.numberSensors;i++)
	{
		errorCode = GetSensorConfiguration(i, &(pSensor+i)->m_config);
		if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
	}
	//////////////////////////////////////////////////////////////////////////////
	// GET TRANSMITTER CONFIGURATION
	pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
	for(i=0;i<ATC3DG.m_config.numberTransmitters;i++)
	{
		errorCode = GetTransmitterConfiguration(i, &(pXmtr+i)->m_config);
		if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
	}
	//////////////////////////////////////////////////////////////////////////////
	// Search for the first attached transmitter and turn it on
	for(id=0;id<ATC3DG.m_config.numberTransmitters;id++) {
		if((pXmtr+id)->m_config.attached) {
			// Transmitter selection is a system function.
			// Using the SELECT_TRANSMITTER parameter we send the id of the
			// transmitter that we want to run with the SetSystemParameter() call
			errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
			if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
			break;
		}
	}
	printf("TrakSTAR Initialized...\n Initializing DirectShow Video Capture \n");
#endif
	//////////////////////////////////////////////////////////////////////////////
	// Initialize opencv stuff
	cvNamedWindow( "ComputerAidedIntubation", CV_WINDOW_AUTOSIZE );
	// set up the image capture
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
	// test if the capture variable points to a meaningfull directshow compatible source
    if ( !capture ) {
        fprintf( stderr, "ERROR: capture is NULL \n" );
        getchar();
        return -1;
    }
	printf("Initialized, starting main loop, press ESC to exit\n");
	/////////////////////////////////////////////////////////////////////////////////
	// Start the openCV loop that grabs images from the directshow capture system
	while ( 1 ) {
        frame = cvQueryFrame( capture );  // check to see if frame is null
        if ( !frame ) {
            fprintf( stderr, "ERROR: frame is null...\n" );
            getchar();
            break;
        }
		// start sensing the birds
#ifdef USE_SENSOR
		if ( count % 20 == 0) {
			sensorID=0;
			// sensor attached so get record
			errorCode = GetAsynchronousRecord(sensorID, pSens0Target, sizeof(sens0Target));
			if(errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(errorCode);}
			// get the status of the last data record
			// only report the data if everything is okay
			unsigned int status = GetSensorStatus( sensorID);
			if( status == VALID_STATUS) {
				sens0TargetTrans = populateTransMat(sens0Target);
			}
			sensorID=1;
			// sensor attached so get record
			errorCode = GetAsynchronousRecord(sensorID, pSens1Scope, sizeof(sens1Scope));
			if(errorCode!=BIRD_ERROR_SUCCESS) {errorHandler(errorCode);}
			status = GetSensorStatus( sensorID);
			if( status == VALID_STATUS)	{
				sens1ScopeTrans = populateTransMat(sens1Scope);
				//cout << sens1ScopeTrans;
			}

					// here is where the logic goes to decide what to cue to the user
			
			sens0TargetTransExt = sens0TargetTrans;
			sens0TargetTransExt.submat(0,3,2,3) = sens0TargetTrans.submat(0,3,2,3) + sens0TargetTrans.submat(0,0,2,0)*tracheaOffset;
			distToTarget = norm( sens0TargetTransExt.submat(0,3,2,3) - sens1ScopeTrans.submat(0,3,2,3), 2);
			//cout << sens0TargetTrans;
			//cout << sens0TargetTransExt;
			imageLocation = headsUpDisplay(sens0TargetTransExt, sens1ScopeTrans, viewingAngle, thetaOffset, endoscopeOffsets);
		}

#endif
		// update clock every 30 frames and set up the plot to the HEADS up display
		if ( count%30 == 0) {
			//sprintf_s(distText, "%5.3f cm", INCH_TO_CM * distToTarget);
			//sprintf_s(distText, "%5.3f deg",  angle * 180 / 3.14 );
			sprintf_s(distText, "%5.3f cm",  distToTarget*INCH_TO_CM );
		}
		// overlay some text

		cvPutText (frame,"(d) Display",cvPoint(3,20), &font, defaultTextColor);
		cvPutText (frame,displayToggleText,cvPoint(3,450), &font, displayTextColor);
		cvPutText (frame,"Press (g) to switch",cvPoint(3,470), &font, displayTextColor);

		if (toggleGuidanceState == 1) {
		// put the overlay on the location of the screen corresponding to the robotics.
			headsUpDisplayOverlay(frame, imageLocation, headsUpLocations, IMAGE_CENTER_X, IMAGE_CENTER_Y, 
								ENDOSCOPE_IMAGE_RADIUS /2 ,ENDOSCOPE_IMAGE_RADIUS );
			
			//for (i = 1; i< 13 ; i++){
		//		headsUpDisplayOverlay(frame, i, headsUpLocations, IMAGE_CENTER_X, IMAGE_CENTER_Y, 
	//							ENDOSCOPE_IMAGE_RADIUS /2 ,ENDOSCOPE_IMAGE_RADIUS ); }


		}


		if (toggleDisplayState == 1) {
			sprintf_s(rollText,"%3.0f degrees", thetaOffset); 
			cvPutText (frame,distText,cvPoint(3,70), &font, defaultTextColor);
			cvPutText (frame,"to Target",cvPoint(3,90), &font, defaultTextColor);
			cvPutText (frame,rollText,cvPoint(3,120), &font, defaultTextColor);
			cvPutText (frame,"(R/r) Roll Correction",cvPoint(3,140), &font, defaultTextColor);
			sprintf_s(viewAngleText,"%3.0f degrees",viewingAngle);
			cvPutText (frame,viewAngleText,cvPoint(3,170), &font, defaultTextColor);
			cvPutText (frame,"(A/a) View Angle",cvPoint(3,190), &font, defaultTextColor);
			sprintf_s(tracheaOffsetText,"%2.1f cm",tracheaOffset*INCH_TO_CM);
			cvPutText (frame,tracheaOffsetText,cvPoint(3,220), &font, defaultTextColor);
			cvPutText (frame,"(O/o) Trachea Offset",cvPoint(3,240), &font, defaultTextColor);
			sprintf_s(yzOffsetText,"Y %1.1fcm, Z %1.1fcm ",endoscopeOffsets(1)*INCH_TO_CM, endoscopeOffsets(2)*INCH_TO_CM);
			cvPutText (frame,yzOffsetText,cvPoint(3,270), &font, defaultTextColor);
			cvPutText (frame,"(Y/y, Z/z) Sensor Offset",cvPoint(3,290), &font, defaultTextColor);
		}
		// plot image
        cvShowImage( "ComputerAidedIntubation", frame );
		// cancel the software if you hold down on the escape key
		keyPress = cvWaitKey(15); // the & 255 is a bitwise AND, so it flattens any bits above 8-bits
		keyPress = ( keyPress & 255);
		if ( keyPress == 27 ) break;  // 27 is the "esc" key to quit guidance
		if ( keyPress == 103) { // 103 is the "g" key for toggle 
			if ( toggleGuidanceState == 1) {
				toggleGuidanceState = 0;
				sprintf_s(displayToggleText, "Guidance OFF");
				displayTextColor = CV_RGB(125,0,0);
			}
			else {
				toggleGuidanceState = 1;
				sprintf_s(displayToggleText, "Guidance ON");
				displayTextColor = CV_RGB(0,125,0);
			}
		}
				if ( keyPress == 100) { // 116 is the "d" key for toggle display
			if ( toggleDisplayState == 1) {
				toggleDisplayState = 0;
			}
			else {
				toggleDisplayState = 1;
			}
		}

		if ( keyPress == 114 ) thetaOffset = thetaOffset - 15; // little 'r' decreases roll offset
		if ( keyPress == 82 ) thetaOffset = thetaOffset + 15;  // big 'R' increases roll offset
		if ( keyPress == 97) viewingAngle = viewingAngle -5;  // little 'a' decreases viewing Angle
		if ( keyPress == 65) viewingAngle = viewingAngle +5;  // big 'A' increases viewing Angle
		if ( keyPress == 111) tracheaOffset = tracheaOffset -0.0393;  // little 'o' trachea offset
		if ( keyPress == 79) tracheaOffset = tracheaOffset + 0.0393;  // Big 'O' trachea offset
		if ( keyPress == 121) endoscopeOffsets(1) = endoscopeOffsets(1) - 0.0393;  // little 'y' decreases y offset
		if ( keyPress == 89) endoscopeOffsets(1) = endoscopeOffsets(1) + 0.0393;;  // big 'Y' increases y offset
		if ( keyPress == 122) endoscopeOffsets(2) = endoscopeOffsets(2) - 0.0393;  // little 'z' decreases offset
		if ( keyPress == 90) endoscopeOffsets(2) = endoscopeOffsets(2) + 0.0393;  // Big 'Z' z offset
		count++;
	}
#ifdef USE_SENSOR
	//////////////////////////////////////////////////////////////////////////////
	// Turn off the transmitter before exiting
	// We turn off the transmitter by "selecting" a transmitter with an id of "-1"
	id = -1;
	errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
	if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode);
	delete[] pXmtr;
	delete[] pSensor;
#endif
	//////////////////////////////////////////////////////////////////////////////
	//  Free memory allocations before exiting
	cvReleaseCapture( &capture );
    cvDestroyWindow("mywindow");
	return 0;
}