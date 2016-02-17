#include <iostream>
#include <cv.h>
#include "highgui.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <Windows.h>

using namespace cv;
using namespace std;

int counter = 0;

vector<Point2f> points;
CvPoint2D32f * pts;

void on_mouse( int event, int x, int y, int flags, void* param )
{
	switch(event)
	{
	case CV_EVENT_LBUTTONDOWN:
		{
			CvPoint seed = cvPoint(x,y);
			pts[counter] = cvPoint2D32f(seed.x, seed.y);
			points.push_back(seed);
			cout << "X: " << seed.x <<" Y: " << seed.y << endl;
			counter++;
			break;
		}
	}
}

int main( int argc, char** argv )
{

	HWND hWnd = GetConsoleWindow();
	ShowWindow( hWnd, SW_HIDE );

	//// Capture the Image from the webcam
	//CvCapture *pCapturedImage = cvCreateCameraCapture(0);

	//// Get the frame
	//IplImage *pSaveImg = cvQueryFrame(pCapturedImage);

	//// Save the frame into a file
	//cvSaveImage("test.jpg" ,pSaveImg);
	IplImage  * resizedImage;
	resizedImage = cvCreateImage(cvSize(512,384),8,3);
	CvCapture *capture = cvCreateCameraCapture(0);
	IplImage  *frame = 0;
	int       key = 0;

	/* initialize camera */
	capture = cvCaptureFromCAM( 0 ); // capture frames from any device it founds

	/* always check */
	if ( !capture ) {
		fprintf( stderr, "Cannot open initialize webcam!\n" );
		system("pause");
		return 1;
	}

	/* create a window for the video */
	cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );

	int counter=1;

	while(( key != 'q') && (counter<300)) {
		/* get a frame */
		frame = cvQueryFrame( capture );

		cvResize(frame,resizedImage,CV_INTER_LINEAR);

		string name, file_name ,type = ".jpg", prefix = "test_";
		stringstream out;
		out << counter;
		name=out.str();
		file_name= prefix + name + type;

		Mat imgMat(resizedImage);  //convert from IplImage to Mat type


		char str1[15];
		char str2[4];
		char *str3 = ".jpg";

		if(counter>9)
		{
			if (counter-9<10)
			{
				strcpy_s (str1,"00000");
				strcpy_s (str2,_itoa(counter-9,str2,10));
				strncat (str1, str2, 7);
			}

			else if(9<counter-9 && counter-9<100)
			{
				strcpy_s (str1,"0000");
				strcpy_s (str2,_itoa(counter-9,str2,10));
				strncat (str1, str2, 7);
			}
			else
			{
				strcpy_s (str1,"000");
				strcpy_s (str2,_itoa(counter-9,str2,10));
				strncat (str1, str2, 7);
			}

			char dir[20];
			strcpy_s(dir, "Image/");

			strncat (str1, str3, 7);
			strncat (dir, str1, 12);

			cvSaveImage(dir,resizedImage);

		}
		//cv::imwrite(file_name,imgMat);

		counter++;

		/* always check */
		if( !frame ) break;

		/* display current frame */
		cvShowImage( "result", frame );

		/* exit if user press 'q' */
		key = cvWaitKey( 50 );  // equivalent of 5 seconds
	}

	/* free memory */
	cvDestroyWindow( "result" );
	//    cvReleaseCapture( &capture );

   system("exit");


	return 0;
}
