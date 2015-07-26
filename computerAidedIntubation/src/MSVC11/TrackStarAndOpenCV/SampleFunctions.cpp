#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <armadillo>
#include <math.h>
#include <stdio.h>
using namespace arma;
//////////////////////////////////////////////////////////////////////////////
//	ERROR HANDLER
//	=============
// This is a simplified error handler.
// This error handler takes the error code and passes it to the GetErrorText()
// procedure along with a buffer to place an error message string.
// This error message string can then be output to a user display device
// like the console
// Specific error codes should be parsed depending on the application.
//////////////////////////////////////////////////////////////////////////////


void errorHandler(int error)
{
	char			buffer[1024];
	char			*pBuffer = &buffer[0];
	int				numberBytes;

	while(error!=BIRD_ERROR_SUCCESS)
	{
		error = GetErrorText(error, pBuffer, sizeof(buffer), SIMPLE_MESSAGE);
		numberBytes = strlen(buffer);
		buffer[numberBytes] = '\n';		// append a newline to buffer
		printf("%s", buffer);
		printf("Press Any Key To Continue! \n You must fix the error for this demo to work");
		getchar();
	}
	exit(0);
}

void drawArrow(IplImage *image, CvPoint p, CvPoint q, CvScalar color, int arrowMagnitude = 9, int thickness=1, int line_type=8, int shift=0) 
{
	double angle = 0;
	double pi = 3.1415;

    //Draw the principle line
    cvLine(image, p, q, color, thickness, line_type, shift);
    //compute the angle alpha
    angle = atan2((double)p.y-q.y, (double)p.x-q.x);
    //compute the coordinates of the first segment
	p.x = (int) ( q.x +  arrowMagnitude * cos(angle + pi/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle + pi/4));
    //Draw the first segment
    cvLine(image, p, q, color, thickness, line_type, shift);
    //compute the coordinates of the second segment
	p.x = (int) ( q.x +  arrowMagnitude * cos(angle - pi/4));
	p.y = (int) ( q.y +  arrowMagnitude * sin(angle - pi/4));
    //Draw the second segment
    cvLine(image, p, q, color, thickness, line_type, shift);
}  


 mat populateTransMat(DOUBLE_POSITION_MATRIX_RECORD inputRecord)
 {
	mat x(4,4);
	x.fill(0); x(3,3) = 1.0;
	x(0,3) = inputRecord.x;
	x(1,3) = inputRecord.y;
	x(2,3) = inputRecord.z;
	x(0,0) = inputRecord.s[0][0];
	x(0,1) = inputRecord.s[1][0];
	x(0,2) = inputRecord.s[2][0];
	x(1,0) = inputRecord.s[0][1];
	x(1,1) = inputRecord.s[1][1];
	x(1,2) = inputRecord.s[2][1];
	x(2,0) = inputRecord.s[0][2];
	x(2,1) = inputRecord.s[1][2];
	x(2,2) = inputRecord.s[2][2];

	return x;
 }

/*
 headsUpDisplay requires the homogeneous transformation matrix of both the target (channel 1 on TrakStar) 
 and scope (channel 2 on TrakStar).  These matrices are both 4x4 matrices.  Viewing angle is the conical
 viewing angle of the endoscope in degrees.
 */
 int headsUpDisplay( mat targetTrans, mat scopeTrans, double viewingAngle, double thetaOffset, vec endoscopeOffsets) 
 {
	double coneRadius = 0;
	double targetRadius = 0;
	vec targetScopeFrame(4); targetScopeFrame.fill(0);
	int headsUpZone = 0;
	double theta = 0;

	targetScopeFrame= scopeTrans.i() * targetTrans.submat(0,3,3,3);
	// X direction is the distance between the target and the scope along the axis of the cylindrical sensor.
	// Treat Y as Y, but Z as X for the scope plane.
	coneRadius = tan(viewingAngle/180*3.14/2)* (targetScopeFrame(0) - endoscopeOffsets(0)) ;
	targetRadius = sqrt ( ( targetScopeFrame(1)-endoscopeOffsets(1)) *(targetScopeFrame(1)-endoscopeOffsets(1)) 
		+ ( targetScopeFrame(2) - endoscopeOffsets(2))* (targetScopeFrame(2)- endoscopeOffsets(2))  );

	theta = atan2(targetScopeFrame(1)-endoscopeOffsets(1), targetScopeFrame(2)- endoscopeOffsets(2) ) * 180.0 / 3.14 + thetaOffset; 
	//printf("coneRadius %f, targetRadius %f, theta %f\n", coneRadius, targetRadius, theta );
	//cout << targetScopeFrame;

	//loop around so that theta goes from -180 to 180 to be consistent with atan2.
	if ( theta > 180 ) // if theta is larger than 180, need to take the remainder from 180 and add it to -180
		theta = -180 + (theta - 180);
	if ( theta < -180 )
		theta = 180 + (theta +180);


	if ( targetRadius < coneRadius/2) {  // straight ahead
		headsUpZone = STRAIGHT_IDX;
		//printf("straight\n");
		}

	// check for the inner heads up display
	if ( ( targetRadius >= coneRadius/2 ) && (targetRadius < coneRadius ) ) {

		if  ( ( theta > -45 ) && (theta <= 45) ) {
			headsUpZone = STRAIGHT_RIGHT_IDX;
			//printf("straight right\n");
		}

		if ( ( theta > 45 ) && (theta <= 135) ) {
			headsUpZone = STRAIGHT_UP_IDX;
			//printf("straight up\n");
		}

		if ( ( theta > 135 ) && (theta <= 180) ) {
			headsUpZone = STRAIGHT_LEFT_IDX;
			//printf("straight left\n");
		}
		if ( ( theta < -135 ) && ( theta >= -180  ) ){
			headsUpZone = STRAIGHT_LEFT_IDX;
			//printf("straight left\n");
		}

		if ( ( theta < -45 ) && ( theta >= -135) ) {
			headsUpZone = STRAIGHT_DOWN_IDX;
			//printf("straight down\n");
		}
	}
	
	// big loop looking for the outer decision
	if  ( targetRadius > coneRadius ) {

		if  ( ( theta > -22.5 ) && (theta <= 22.5) ) {
			headsUpZone = RIGHT_IDX;
			//printf("right\n");
		}

		if  ( ( theta > 22.5 ) && (theta <= 67.5) ) {
			headsUpZone = UP_RIGHT_IDX;
			//printf("up right\n");
		}

		if  ( ( theta > 67.5 ) && (theta <= 112.5) ) {
			headsUpZone = UP_IDX;
			//printf("up \n");
		}

		if  ( ( theta > 112.5 ) && (theta <= 157.5) ) {
			headsUpZone = UP_LEFT_IDX;
			//printf("up left\n");
		}

		if  ( ( theta > 157.5 ) && (theta <= 180) ) {
			headsUpZone = LEFT_IDX;
			//printf("left\n");
		}

		if  ( ( theta > -180 ) && (theta <= -157.5) ) {
			headsUpZone = LEFT_IDX;
			//printf("left\n");
		}

		if  ( ( theta > -157.5 ) && (theta <= -112.5) ) {
			headsUpZone = DOWN_LEFT_IDX;
			//printf("down left\n");
		}

		if  ( ( theta > -112.5 ) && (theta <= -67.5) ) {
			headsUpZone = DOWN_IDX;
			//printf("down\n");
		}

		if  ( ( theta > -67.5 ) && (theta <= -22.5) ) {
			headsUpZone = DOWN_RIGHT_IDX;
			//printf("down right\n");
		}

	}
	
	return headsUpZone;
 }

void headsUpDisplayOverlay(IplImage * frm, int imageLocat, mat hUpLoc, int centerX, int centerY, int radiusIn, int radiusOut)
{
	int      lineWidth= 2;
	int		 lineType = 8;
	int		multX = 1;
	int		multY = 1;
	int     thetaStart = 0;
	int		thetaEnd = 0;

	if ( (imageLocat == STRAIGHT_DOWN_IDX ) | 
		( imageLocat == STRAIGHT_RIGHT_IDX ) | ( imageLocat == STRAIGHT_LEFT_IDX ) |
		( imageLocat == STRAIGHT_UP_IDX ) ){

		if (imageLocat == STRAIGHT_DOWN_IDX){
			CvPoint  curve1[]={centerX+ 0.707*radiusIn,centerY + 0.707*radiusIn ,  centerX+ 0.707*radiusOut,centerY + 0.707*radiusOut}; 
			CvPoint  curve2[]={centerX - 0.707*radiusIn,centerY + 0.707*radiusIn ,  centerX - 0.707*radiusOut,centerY + 0.707*radiusOut}; 
			CvPoint* curveArr[2]={curve1, curve2};
			int      nCurvePts[2]={2,2};
			int      nCurves=2;
			int      isCurveClosed=0;
			cvPolyLine(frm,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(0,0,255),lineWidth);  //draw the line
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusIn, radiusIn ), 0, 45,135, cvScalar(0,255,255),lineWidth,lineType ); // inner ellispe
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusOut, radiusOut ), 0, 45,135, cvScalar(0,255,255),lineWidth,lineType ); // outer ellipse
			drawArrow(frm,  cvPoint(centerX , centerY + 150), cvPoint(centerX , centerY  + 180 ), cvScalar(0,255,255), 9, 2, 8, 0);
		}
		if (imageLocat == STRAIGHT_UP_IDX){
			CvPoint  curve1[]={centerX+ 0.707*radiusIn, centerY - 0.707*radiusIn , centerX+ 0.707*radiusOut, centerY - 0.707*radiusOut}; 
			CvPoint  curve2[]={centerX - 0.707*radiusIn,centerY - 0.707*radiusIn ,  centerX - 0.707*radiusOut,centerY - 0.707*radiusOut}; 
			CvPoint* curveArr[2]={curve1, curve2};
			int      nCurvePts[2]={2,2};
			int      nCurves=2;
			int      isCurveClosed=0;
			cvPolyLine(frm,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(0,255,255),lineWidth);  //draw the line
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusIn, radiusIn ), 0, 225,315, cvScalar(0,255,255),lineWidth,lineType ); // inner ellispe
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusOut, radiusOut ), 0, 225,315, cvScalar(0,255,255),lineWidth,lineType ); // outer ellipse
			drawArrow(frm,  cvPoint(centerX , centerY - 150), cvPoint(centerX , centerY -180 ), cvScalar(0,255,255), 9, 2, 8, 0);
		}
		if (imageLocat == STRAIGHT_RIGHT_IDX){
			CvPoint  curve1[]={centerX+ 0.707*radiusIn,centerY - 0.707*radiusIn ,  centerX+ 0.707*radiusOut,centerY - 0.707*radiusOut}; 
			CvPoint  curve2[]={centerX + 0.707*radiusIn,centerY + 0.707*radiusIn ,  centerX + 0.707*radiusOut,centerY + 0.707*radiusOut}; 
			CvPoint* curveArr[2]={curve1, curve2};
			int      nCurvePts[2]={2,2};
			int      nCurves=2;
			int      isCurveClosed=0;
			cvPolyLine(frm,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(0,255,255),lineWidth);  //draw the line
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusIn, radiusIn ), 0, -45,45, cvScalar(0,255,255),lineWidth,lineType ); // inner ellispe
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusOut, radiusOut ), 0, -45,45, cvScalar(0,255,255),lineWidth,lineType ); // outer ellipse

			drawArrow(frm,  cvPoint(centerX + 150, centerY ), cvPoint(centerX + 180, centerY ), cvScalar(0,255,255), 9, 2, 8, 0);
		}
		if (imageLocat == STRAIGHT_LEFT_IDX){
			CvPoint  curve1[]={centerX- 0.707*radiusIn,centerY - 0.707*radiusIn ,  centerX- 0.707*radiusOut,centerY - 0.707*radiusOut}; 
			CvPoint  curve2[]={centerX - 0.707*radiusIn,centerY + 0.707*radiusIn ,  centerX - 0.707*radiusOut,centerY + 0.707*radiusOut}; 
			CvPoint* curveArr[2]={curve1, curve2};
			int      nCurvePts[2]={2,2};
			int      nCurves=2;
			int      isCurveClosed=0;
			cvPolyLine(frm,curveArr,nCurvePts,nCurves,isCurveClosed,cvScalar(0,255,255),lineWidth);  //draw the line
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusIn, radiusIn ), 0, 135,225, cvScalar(0,255,255),lineWidth,lineType ); // inner ellispe
			cvEllipse( frm, cvPoint( centerX, centerY ), cvSize( radiusOut, radiusOut ), 0, 135,225, cvScalar(0,255,255),lineWidth,lineType ); // outer ellipse
			drawArrow(frm,  cvPoint(centerX - 150, centerY ), cvPoint(centerX - 180, centerY ), cvScalar(0,255,255), 9, 2, 8, 0);
		}

	}
	if ( imageLocat == STRAIGHT_IDX ) 
		cvCircle( frm, cvPoint(centerX,centerY),radiusIn-3, cvScalar( 0, 255, 0 ), lineWidth, lineType ); // quick hack to remove 5 pix


	if ( (imageLocat == DOWN_IDX ) | ( imageLocat == RIGHT_IDX ) | ( imageLocat == LEFT_IDX ) | ( imageLocat == UP_IDX ) |
		(imageLocat == UP_LEFT_IDX) | ( imageLocat == UP_RIGHT_IDX) | ( imageLocat == DOWN_RIGHT_IDX) | (imageLocat == DOWN_LEFT_IDX) ){
			//CvFont	font;
			//cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1.0,1.0,0,2.0);

			CvPoint orig; orig = cvPoint(centerX, centerY);
			CvPoint draw; draw = cvPoint((int) hUpLoc(imageLocat,0) , (int) hUpLoc(imageLocat,1) );
			CvPoint vec; vec = cvPoint((int) hUpLoc(imageLocat,0) - centerX, (int) hUpLoc(imageLocat,1) - centerY);

			drawArrow(frm,  cvPoint( ((int) hUpLoc(imageLocat,0) - 0.1*vec.x ) , ((int) hUpLoc(imageLocat,1) - 0.1*vec.y ) ),
				cvPoint((int) hUpLoc(imageLocat,0) , (int) hUpLoc(imageLocat,1) ) , cvScalar(0,0,255), 9, 2, 8, 0);


			//drawArrow(frm,  orig/*cvPoint((int) hUpLoc(imageLocat,0) , (int) hUpLoc(imageLocat,1) )*/,
			//	cvPoint((int) hUpLoc(imageLocat,0) , (int) hUpLoc(imageLocat,1) ) , cvScalar(0,0,255), 9, 2, 8, 0);
			//cvPutText (frm,"O",cvPoint( (int) hUpLoc(imageLocat,0), (int) hUpLoc(imageLocat,1) ), &font, cvScalar(0,255,0));

	}
}

mat createHeadsUpCoords(int centX, int centY, double imageRad, double mult)
 {
	mat headsUpLoc(NUM_HEADSUP_COORDS,2); 

	/*
	headsUpLoc.fill(0);
	headsUpLoc(UP_IDX ,0) = 720/2;				headsUpLoc(UP_IDX,1) = 10; 
	headsUpLoc(DOWN_IDX ,0) = 720/2;			headsUpLoc( DOWN_IDX,1) = 450; 
	headsUpLoc(LEFT_IDX,0) = 100;				headsUpLoc(LEFT_IDX,1) = 480/2; 
	headsUpLoc(RIGHT_IDX,0) = 600;			headsUpLoc(RIGHT_IDX,1) = 480/2;
	headsUpLoc(UP_RIGHT_IDX,0) = 550;			headsUpLoc(UP_RIGHT_IDX,1) = 50; 
	headsUpLoc(DOWN_RIGHT_IDX,0) = 550;		headsUpLoc(DOWN_RIGHT_IDX,1) = 400; 
	headsUpLoc(UP_LEFT_IDX,0) = 200;			headsUpLoc(UP_LEFT_IDX,1) = 50; 
	headsUpLoc(DOWN_LEFT_IDX,0) = 200;		headsUpLoc(DOWN_LEFT_IDX,1) = 400;
	headsUpLoc(STRAIGHT_IDX,0) = 720/2;		headsUpLoc(STRAIGHT_IDX,1) = 480/2; 
	headsUpLoc(STRAIGHT_LEFT_IDX,0) = 200;	headsUpLoc(STRAIGHT_LEFT_IDX,1) = 480/2; 
	headsUpLoc(STRAIGHT_RIGHT_IDX,0) = 500;	headsUpLoc(STRAIGHT_RIGHT_IDX,1) = 480/2; 
	headsUpLoc(STRAIGHT_UP_IDX,0) = 720/2;	headsUpLoc(STRAIGHT_UP_IDX,1) = 100; 
	headsUpLoc(STRAIGHT_DOWN_IDX,0) = 720/2;	headsUpLoc(STRAIGHT_DOWN_IDX,1) = 350; */

	headsUpLoc(UP_IDX ,0) = centX;				
	if (centY - imageRad < 0)
		headsUpLoc(UP_IDX,1) = 1;
	else
		headsUpLoc(UP_IDX,1) = centY-imageRad;


	headsUpLoc(DOWN_IDX ,0) = centX;
	if (centY + imageRad > 479)
		headsUpLoc( DOWN_IDX,1) = 479;
	else
		headsUpLoc( DOWN_IDX,1) = centY+imageRad; 


	headsUpLoc(LEFT_IDX,0) = centX -imageRad;	headsUpLoc(LEFT_IDX,1) = centY; 
	headsUpLoc(RIGHT_IDX,0) = centX + imageRad;	headsUpLoc(RIGHT_IDX,1) = centY;

	headsUpLoc(UP_RIGHT_IDX,0) = centX + mult*imageRad*cos(-PI/4);
	headsUpLoc(UP_RIGHT_IDX,1) = centY + mult*imageRad*sin(-PI/4); 
	
	headsUpLoc(DOWN_RIGHT_IDX,0) = centX + mult*imageRad*cos(PI/4);
	headsUpLoc(DOWN_RIGHT_IDX,1) = centY + mult*imageRad*sin(PI/4);  
	
	headsUpLoc(UP_LEFT_IDX,0) = centX + mult*imageRad*cos(-PI*3/4);		
	headsUpLoc(UP_LEFT_IDX,1) = centY + mult*imageRad*sin(-PI*3/4); 
	
	headsUpLoc(DOWN_LEFT_IDX,0) = centX + mult*imageRad*cos(PI*3/4);		
	headsUpLoc(DOWN_LEFT_IDX,1) = centY + mult*imageRad*sin(PI*3/4);
	
	headsUpLoc(STRAIGHT_IDX,0) = 720/2;		headsUpLoc(STRAIGHT_IDX,1) = 480/2; 
	headsUpLoc(STRAIGHT_LEFT_IDX,0) = 200;	headsUpLoc(STRAIGHT_LEFT_IDX,1) = 480/2; 
	headsUpLoc(STRAIGHT_RIGHT_IDX,0) = 500;	headsUpLoc(STRAIGHT_RIGHT_IDX,1) = 480/2; 
	headsUpLoc(STRAIGHT_UP_IDX,0) = 720/2;	headsUpLoc(STRAIGHT_UP_IDX,1) = 100; 
	headsUpLoc(STRAIGHT_DOWN_IDX,0) = 720/2;	headsUpLoc(STRAIGHT_DOWN_IDX,1) = 350;


//	cout << headsUpLoc;

	return headsUpLoc;
 }
 /*
 This returns the angle in radians between the vector between the X axis of the scope and the position of the target
 the inputs are two matrices 4x4 homogeneous transformation matrices for the two sensor channels.  Original code was:
 angle = acos( dot(  sens1ScopeTrans.submat(0,3,2,3) - sens0TargetTrans.submat(0,3,2,3) , sens1ScopeTrans.submat(0,0,2,0) ) / distToTarget );
 */
 double targetConicalAngle(mat targetTrans, mat scopeTrans)
 {
	double angle = 0;
	double distToTarget = 0;
	distToTarget = norm( targetTrans.submat(0,3,2,3) - scopeTrans.submat(0,3,2,3), 2);
	angle = acos( dot(  scopeTrans.submat(0,3,2,3) - targetTrans.submat(0,3,2,3) , scopeTrans.submat(0,0,2,0) ) / distToTarget );
	return angle;
 }
