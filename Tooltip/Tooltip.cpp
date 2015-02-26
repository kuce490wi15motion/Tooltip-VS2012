// Tooltip.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <exception>
#include <DepthSense.hxx>
#include "tchar.h"
#include "SerialClass.h"	// Library described above
#include "string"
#include "math.h"
//#include "serial.cpp"
#include "ToolData.cpp"

#ifdef _MSC_VER
#include <windows.h>
#endif

//namespace usage
using namespace DepthSense;
using namespace cv;
using namespace std;

//Global Variables
	int16_t leftDepth;
	int16_t rightDepth;

//Gobal Vairables for Arduino
	Serial* SP;
	char* sendCharacter = "!";
	char incomingData[36] = "";			
	// don't forget to pre-allocate memory
	//printf("%s\n",incomingData);
	int dataLength = 36;
	int readResult = 0;


//DS325 Nodes
Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

//DS325 Total Captured Frames
uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

//DS325 Reference Frame
CvSize g_szDepth=cvSize(320,240);
IplImage* img = cvLoadImage("DefaultSpace.jpg", 1);
IplImage* g_depthImage = NULL; 
Mat prevFrame(img);


//Global Frames
Mat livefeed;
Mat depthfeed;
Mat depthFrame;
//g_szVideo=cvSize(1280,720); //WXGA_H

bool g_bDeviceFound = false;

int depthVal =0;
double toolArea = 90000;

//Struct data for Tracking & Sensing
int dataPosition =0;
ToolData GripperData [10000];
ToolData GripperExpert [10000];

//objects to track. Green LH Gripper
int16_t gripperLeft[3] = {0,0,0};
float leftTool[3] = {0,0,0};
//objects to track. Red RH Gripper
int16_t gripperRight[3] = {0,0,0};
float rightTool[3] = {0,0,0};


/*//control variables for threshold adjustment
int iLowH = 0;
int iHighH = 179;

int iLowS = 0; 
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;
*/

//HSV values for Red (RH Gripper) 
int iLowH = 160;
int iHighH = 179;

int iLowS = 40; 
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

//HSV values for Green (LH Gripper)
int iLowH2 = 46;
int iHighH2 = 60;

int iLowS2 = 40; 
int iHighS2 = 255;

int iLowV2 = 40;
int iHighV2 = 255;

//our sensitivity value to be used in the threshold() function
const static int SENSITIVITY_VALUE = 127;
//size of blur used to smooth the image to remove possible noise and
//increase the size of the object we are trying to track. 
const static int BLUR_SIZE = 10;

//bounding rectangle of the object, we will use the center of this as its position.
Rect gLBoundingRectangle = Rect(0,0,0,0);
Rect gRBoundingRectangle = Rect(0,0,0,0);

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

//int to string helper function
string intToString(int number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//tracking function to find objects
void searchForMovement(Mat thresholdImage, Mat &cameraFeed, char toolType){
	int x,y =0;
	//Scalar color =(0,0,0);
	//notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
	//to take the values passed into the function and manipulate them, rather than just working with a copy.
	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//Calculate the moments of the thresholded image
	Moments oMoments = moments(temp);

   double dM01 = oMoments.m01;
   double dM10 = oMoments.m10;
   double dArea = oMoments.m00;

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
   if (dArea > toolArea) {
		//calculate the position of the ball
	   double xpos = dM10 / dArea;
	   double ypos = dM01 / dArea;   

	   //update the objects positions by changing the 'gripperLeft' or 'gripperRight' array values
		if (toolType == 'R') {
			gripperRight[0] = (int) xpos , gripperRight[1] = (int) ypos;
			GripperData[dataPosition].rightXPos = (float) xpos;
			GripperData[dataPosition].rightYPos = (float) ypos;
			GripperData[dataPosition].rightZPos  = (float) gripperRight[2];
			//GripperData[dataPosition].
	}
		if (toolType == 'L') {
			gripperLeft[0] = (int) xpos , gripperLeft[1] = (int) ypos;
			GripperData[dataPosition].leftXPos = (float) xpos;
			GripperData[dataPosition].leftYPos = (float) ypos;
			GripperData[dataPosition].leftZPos  = (float) gripperLeft[2];
			//GripperData[dataPosition].
			dataPosition++;
		}

		//make some temp x and y variables so we dont have to type out so much
	if (toolType == 'R') {
		x = gripperRight[0];
		y = gripperRight[1];
		depthVal = gripperRight[2];
	}
	if (toolType == 'L') {
		x = gripperLeft[0];
		y = gripperLeft[1];
		depthVal = gripperLeft[2];
	}
	
	//draw some crosshairs around the object
	circle(cameraFeed,Point(x,y),10,Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x,y-11),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x,y+11),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x-11,y),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x+11,y),Scalar(0,255,0),2);
	
	if (toolType == 'R') {

		//write the position of the object to the screen
		putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+","+intToString(depthVal)+")",Point(x,y),1,1,Scalar(0,0,255),2);
	}
	if (toolType == 'L') {

		//write the position of the object to the screen
		putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+","+intToString(depthVal)+")",Point(x,y),1,1,Scalar(20,255,0),2);
	}

   }
   /*
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours

	//if contours vector is not empty, we have found some objects
	if(contours.size()>0)
		objectDetected=true;
	else 
		objectDetected = false;
		
	if(objectDetected){
		//the largest contour is found at the end of the contours vector
		//we will simply assume that the biggest contour is the object we are looking for.
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size()-1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		gLBoundingRectangle = boundingRect(largestContourVec.at(0));
		int xpos = gLBoundingRectangle.x;
		int ypos = gLBoundingRectangle.y;

		//update the objects positions by changing the 'gripperLeft' or 'gripperRight' array values
		if (toolType == 'R') {
		gripperRight[0] = xpos , gripperRight[1] = ypos;
	}
		if (toolType == 'L') {
			gripperLeft[0] = xpos , gripperLeft[1] = ypos;
		}
	}
	//make some temp x and y variables so we dont have to type out so much
	if (toolType == 'R') {
		x = gripperRight[0];
		y = gripperRight[1];
		depthVal = gripperRight[2];
	}
	if (toolType == 'L') {
		x = gripperLeft[0];
		y = gripperLeft[1];
		depthVal = gripperLeft[2];
	}
	
	//draw some crosshairs around the object
	circle(cameraFeed,Point(x,y),20,Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
	line(cameraFeed,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
	Scalar color = (0,0,0);

	if (toolType == 'R') {
		color = (255,0,0);
	}
	if (toolType == 'L') {
		color = (20,20,255);
	}
	//write the position of the object to the screen
	putText(cameraFeed,"Tracking object at (" + intToString(x)+","+intToString(y)+","+intToString(depthVal)+")",Point(x,y),1,1,color,2);
	*/

}

/*----------------------------------------------------------------------------*/
void getSensorData() {
	if(SP->IsConnected())
	{
		SP->WriteData(sendCharacter,8);
		readResult = SP->ReadData(incomingData,dataLength);
		//printf("Bytes read: (-1 means no data available) %i\n",readResult);

		//readresult value will change!!!
		if (readResult == 35) {
			union {float f1;char b[4];} xAccel;
			xAccel.b[3] = incomingData[4];
			xAccel.b[2] = incomingData[3];
			xAccel.b[1] = incomingData[2];
			xAccel.b[0] = incomingData[1];
		
			union {float f2;char b[4];} yAccel;
			yAccel.b[3] = incomingData[8];
			yAccel.b[2] = incomingData[7];
			yAccel.b[1] = incomingData[6];
			yAccel.b[0] = incomingData[5];

			union {float f3;char b[4];} zAccel;
			zAccel.b[3] = incomingData[12];
			zAccel.b[2] = incomingData[11];
			zAccel.b[1] = incomingData[10];
			zAccel.b[0] = incomingData[9];

			union {float f4;char b[4];} xGyro;
			xGyro.b[3] = incomingData[17];
			xGyro.b[2] = incomingData[16];
			xGyro.b[1] = incomingData[15];
			xGyro.b[0] = incomingData[14];
		
			union {float f5;char b[4];} yGyro;
			yGyro.b[3] = incomingData[21];
			yGyro.b[2] = incomingData[20];
			yGyro.b[1] = incomingData[19];
			yGyro.b[0] = incomingData[18];

			union {float f6;char b[4];} zGyro;
			zGyro.b[3] = incomingData[25];
			zGyro.b[2] = incomingData[24];
			zGyro.b[1] = incomingData[23];
			zGyro.b[0] = incomingData[22];

			union {int i1;char b[4];} rotary1;
			rotary1.b[3] = incomingData[32];
			rotary1.b[2] = incomingData[31];
			rotary1.b[1] = incomingData[30];
			rotary1.b[0] = incomingData[29];


			GripperData[dataPosition].rightAccel = sqrt(xAccel.f1 * xAccel.f1 + yAccel.f2 * yAccel.f2 + zAccel.f3 * zAccel.f3);

			printf("\n Acceleration Right Tool: %f\n",GripperData[dataPosition].rightAccel);
			//get gripper status

			//get encoder status

			//get gyro status

		}




	}

}
/*----------------------------------------------------------------------------*/
float analysis() {
	//float totalResult,tooltipResult,senseResult,bodyResult,taskResult =0;
	float distanceToolL=0.0;;
	float distanceToolR=0.0;
	float accelL=0.0;
	float accelR=0.0;
	float totalResult=0.0;
	int length= sizeof(GripperData)/sizeof(ToolData);
	for (int i=0;i <length;i++) {

		distanceToolL += sqrt(pow((GripperData[i].leftXPos- GripperExpert[i].leftXPos),2)+pow((GripperData[i].leftYPos- GripperExpert[i].leftYPos),2)+pow((GripperData[i].leftZPos- GripperExpert[i].leftZPos),2));
		distanceToolR += sqrt(pow((GripperData[i].rightXPos- GripperExpert[i].rightXPos),2)+pow((GripperData[i].rightYPos- GripperExpert[i].rightYPos),2)+pow((GripperData[i].rightZPos- GripperExpert[i].rightZPos),2));

		accelL+=GripperData[i].leftAccel;
		accelR+=GripperData[i].rightAccel;

	}
	//more analysis needed
	return totalResult;
}
/*----------------------------------------------------------------------------*/
// New audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
	//Do Nothing. Audio not needed.
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
   // printf("C#%u: %d\n",g_cFrames,data.colorMap.size());
    g_cFrames++;
	int32_t w, h;
	Mat modFrame,graymat,graymod,threshFrame,finalFrame;
	w=1280;
	h=720;
	FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);

	if (data.colorMap != nullptr ) {
		cv::Mat livefeed(h,w,CV_8UC3,(void*)(const uint8_t*)data.colorMap);   
		cv::cvtColor(livefeed,graymat,COLOR_BGR2GRAY);
		cv::cvtColor(prevFrame,graymod,COLOR_BGR2GRAY);
		cv::absdiff(graymod,graymat,modFrame);
		//cv::threshold(modFrame,threshFrame,SENSITIVITY_VALUE,255,THRESH_BINARY);
		cv::adaptiveThreshold(modFrame,threshFrame,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);
		cv::blur(threshFrame,threshFrame,cv::Size(BLUR_SIZE,BLUR_SIZE));
		cv::adaptiveThreshold(modFrame,threshFrame,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);
		//cv::threshold(threshFrame,threshFrame,SENSITIVITY_VALUE,255,THRESH_BINARY);

		finalFrame = livefeed.clone();
		//searchForMovement(threshFrame,finalFrame);
		
		Mat imgHSV;
		cvtColor(livefeed, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		Mat imgThresholded, imgThresholded2;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		searchForMovement(imgThresholded,finalFrame,'R');

		inRange(imgHSV, Scalar(iLowH2, iLowS2, iLowV2), Scalar(iHighH2, iHighS2, iHighV2), imgThresholded2); //Threshold the image
		
		
		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		
		searchForMovement(imgThresholded2,finalFrame,'L');
		getSensorData();

		imshow("Thresholded Image Red", imgThresholded); //show the thresholded image
		imshow("Thresholded Image Green", imgThresholded2); //show the thresholded image
		//imshow("Thresholded Image Before", imgHSV); //show the thresholded image
		prevFrame = livefeed.clone();

		//show displays
		//cv::imshow("Original Display",livefeed); 
		//cv::imshow("Threshold Display",threshFrame); 
		cv::imshow("Tracking Display",finalFrame); 

		//cv::imshow("Difference Display",modFrame); 
		//cvCreateImage(g_szVideo,IPL_DEPTH_8U,3);
		char key = cvWaitKey(1);
		if (key==27) {
			printf("Quitting main loop from OpenCV\n");
			g_context.quit();
		}
	}
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
    // Project some 3D points in the Color Frame
    if (!g_pProjHelper)
    {
        g_pProjHelper = new ProjectionHelper (data.stereoCameraParameters);
        g_scp = data.stereoCameraParameters;
    }
    else if (g_scp != data.stereoCameraParameters)
    {
        g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
        g_scp = data.stereoCameraParameters;
    }

    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);
    int cx = w/2;
    int cy = h/2;

    Vertex p3DPoints[4];

    p3DPoints[0] = data.vertices[(cy-h/4)*w+cx-w/4];
    p3DPoints[1] = data.vertices[(cy-h/4)*w+cx+w/4];
    p3DPoints[2] = data.vertices[(cy+h/4)*w+cx+w/4];
    p3DPoints[3] = data.vertices[(cy+h/4)*w+cx-w/4];
    
    Point2D p2DPoints[4];
    g_pProjHelper->get2DCoordinates ( p3DPoints, p2DPoints, 4, CAMERA_PLANE_COLOR);
  
	// show depth image
	if ( data.depthMap != nullptr ) {
		//cv::Mat mat( h, w, CV_16SC1, (void*)(const int16_t*)data.depthMap );
		cv::Mat depthfeed(h,w,CV_16SC1,(void*)(const int16_t*)data.depthMap);   
		//cv::Mat mat( h, w, CV_32SC1, (void*)(const float*)data.depthMapFloatingPoint); 
		//cvShowImage("Depth Camera", cvCloneImage(&(IplImage)mat));

		int count=0; // DS data index
		for (int i=0; i <h;i++) {
			for (int j=0; j<w;j++) {
				int16_t val = data.depthMap[count];
				cvSet2D(g_depthImage,i,j,cvScalar(val));
				count++;
			}
		}
		Mat depthRR(g_depthImage);



		//cv::waitKey(1);
		double xDL =ceil(gripperLeft[0]/4);
		double yDL =ceil(gripperLeft[1]/3);
		double xDR =ceil(gripperRight[0]/4);
		double yDR =ceil(gripperRight[1]/3);
		//int16_t leftDepth = (depthfeed.at<int16_t>(yDL,xDL));
		//int16_t rightDepth = (depthfeed.at<int16_t>(yDR,xDR));


		leftDepth = (depthRR.at<int16_t>(yDL,xDL));
		rightDepth = (depthRR.at<int16_t>(yDR,xDR));

		//check for valid depth values only
		if (leftDepth >=0 && leftDepth <= 31999) {
			gripperLeft[2] = leftDepth;
		}

		if (rightDepth >=0 && rightDepth <= 31999) {
		gripperRight[2] = rightDepth;
		}
		cv::imshow("Depth Camera", depthRR);
	}

	   g_dFrames++;

    // Quit the main loop after 7500 depth frames received
    if (g_dFrames == 7500)
        g_context.quit();
}

/*// old depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
int32_t w, h;
FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);

g_dFrames++;

// show depth image
if ( data.depthMap != nullptr ) {
    //cv::Mat mat( h, w, CV_16SC1, (void*)(const int16_t*)data.depthMap );        
	//cv::Mat mat( h, w, CV_16SC1, (void*)(const float*)data.depthMap ); 
	cv::Mat mat(h, w, CV_16UC1, (void*)(const void*)(data.depthMap));
    cv::imshow( "SoftKinetic Depth Camera", mat );
    cv::waitKey( 1 );
}

  // save depthmap (h =320, w=240)

 //cv::Mat test_mat( h, w, CV_16SC1, (void*)(const int16_t*)data.depthMap );
 cv::Mat test_mat(h, w, CV_16UC1, (void*)(const void*)(data.depthMap));
 cout << "rows: " << test_mat.rows << endl;
 cout << "col: " << test_mat.cols << endl;
 printf("%d \n",test_mat.at<short>(0,0));


}*/

/*----------------------------------------------------------------------------*/
void configureAudioNode()
{
    g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

    AudioNode::Configuration config = g_anode.getConfiguration();
    config.sampleRate = 44100;

    try 
    {
        g_context.requestControl(g_anode,0);

        g_anode.setConfiguration(config);
        
        g_anode.setInputMixerLevel(0.5f);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
    g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_QVGA;
    config.framerate = 25;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = false;

	g_dnode.setEnableDepthMap(true);
    g_dnode.setEnableVertices(true);
	g_dnode.setEnableDepthMapFloatingPoint(false);

    try 
    {
        g_context.requestControl(g_dnode,0);

        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_WXGA_H;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_60HZ;
    config.framerate = 25;

    g_cnode.setEnableColorMap(true);

    try 
    {
        g_context.requestControl(g_cnode,0);

        g_cnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }

    if ((node.is<AudioNode>())&&(!g_anode.isSet()))
    {
        //g_anode = node.as<AudioNode>();
        //configureAudioNode();
       //g_context.registerNode(node);
    }
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
        g_anode.unset();
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
        g_cnode.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
        g_dnode.unset();
    printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
void sensorConnect() {
	SP = new Serial("COM4");    // adjust as needed
	if (SP->IsConnected())
		printf("We're connected\n");





}
/*----------------------------------------------------------------------------*/


int main(int argc, char* argv[])
{
    g_context = Context::create("localhost");
    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    // Get the list of currently connected devices
    vector<Device> da = g_context.getDevices();

    // We are only interested in the first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;

        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

        vector<Node> na = da[0].getNodes();
        
        //printf("Found %u nodes\n",na.size());
        
        for (int n = 0; n < (int)na.size();n++)
            configureNode(na[n]);
    }   

	//HSV adjustment, if needed. FOR EXPERIMENTAL ANALYSIS ONLY
	/*
	const string Control ="Control";
	namedWindow(Control, CV_WINDOW_AUTOSIZE); //create a window called "Control"

	//Create trackbars in "Control" window
	
	cvCreateTrackbar("LowH","Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH","Control", &iHighH, 179);

	cvCreateTrackbar("LowS","Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS","Control", &iHighS, 255);

	cvCreateTrackbar("LowV","Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV","Control", &iHighV, 255);
	
	
	cvCreateTrackbar("LowH2","Control", &iLowH2, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH2","Control", &iHighH2, 179);

	cvCreateTrackbar("LowS2","Control", &iLowS2, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS2","Control", &iHighS2, 255);

	cvCreateTrackbar("LowV2","Control", &iLowV2, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV2","Control", &iHighV2, 255);
	*/
	sensorConnect();
	g_depthImage = cvCreateImage(g_szDepth,IPL_DEPTH_16S,1);
    g_context.startNodes();
    g_context.run();
    g_context.stopNodes();

    if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
    if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
    if (g_anode.isSet()) g_context.unregisterNode(g_anode);

    if (g_pProjHelper)
        delete g_pProjHelper;

	//getchar();
	analysis();
	getchar();
    return 0;
}

