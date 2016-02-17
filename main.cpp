#define _WIN32_WINNT 0x0500
#include <iostream>
#include <cv.h>
#include "highgui.h"
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include "Globals.h"
#include "3DScanner.h"
#include <Windows.h>

using namespace cv;
using namespace std;

int counter = 0;
int extCounter = 0;

Scanner3D * scanner;

vector<Point2f> points;
CvPoint2D32f * pts;
CvPoint2D32f * extPts;

void on_mouse( int event, int x, int y, int flags, void* param )
{
	switch(event)
	{
	case CV_EVENT_LBUTTONDOWN:
		{
			CvPoint seed = cvPoint(x,y);

			if(counter < 6)
			{
				scanner->Points.push_back(seed);
				counter++;
			}
			else
			{
				if(extCounter < 4)
					scanner->extPointsH[extCounter] = cvPoint2D32f(seed.x, seed.y);
				//extPts[counter] = cvPoint2D32f(seed.x, seed.y);

				if(extCounter >= 4 && extCounter <8)
					scanner->extPointsV[extCounter - 4] = cvPoint2D32f(seed.x, seed.y);
				//extPts[counter] = cvPoint2D32f(seed.x, seed.y);

				extCounter++;
			}

			//points.push_back(seed);

			cout << "X: " << seed.x <<" Y: " << seed.y << endl;

			break;
		}
	}
}

int main( int argc, char** argv )
{
	
	HWND hWnd = GetConsoleWindow(); 
	ShowWindow( hWnd, SW_HIDE ); 
	
	scanner = new Scanner3D();
	scanner->readFrameIndex();

	extPts = new CvPoint2D32f[4];
	pts = new CvPoint2D32f[4];

    int numLineCorners  = 2;
    int numBoardCorners = 2;
    int mouseParam = 0;

    //Loading an image
    IplImage* img = cvLoadImage( "Image/000001.jpg");

	IplImage * grayImg = rgb2gray(img);

    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);
    cvNamedWindow( "Select Points", CV_WINDOW_AUTOSIZE );
    cvPutText(img,"Dividing line section" , cvPoint(180,20),&font, cvScalar(0,0,0,0)); //center the text and write it with black color


    cvSetMouseCallback( "Select Points", on_mouse, &mouseParam );
    cvShowImage( "Select Points", img );

    cvWaitKey(0);
    cvDestroyWindow( "Select Points" );
	cvReleaseImage( &img );

    img = cvLoadImage( "Image/000001.jpg");
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);
    cvNamedWindow( "Select Points", CV_WINDOW_AUTOSIZE );
    cvPutText(img,"Choose Vertical Plane Corners" , cvPoint(150,20),&font, cvScalar(0,0,0,0)); //center the text and write it with black color
	cvPutText(img,"  (Top-left & Bottom-Right)" , cvPoint(150,40),&font, cvScalar(0,0,0,0));

    cvSetMouseCallback( "Select Points", on_mouse, &mouseParam );
    cvShowImage( "Select Points", img );

    cvWaitKey(0);
    cvDestroyWindow( "Select Points" );
	cvReleaseImage( &img );

	img = cvLoadImage( "Image/000001.jpg");
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);
    cvNamedWindow( "Select Points", CV_WINDOW_AUTOSIZE );
    cvPutText(img,"Choose Horizontal Plane Corners" , cvPoint(150,20),&font, cvScalar(0,0,0,0)); //center the text and write it with black color
	cvPutText(img,"  (Top-left & Bottom-Right)" , cvPoint(150,40),&font, cvScalar(0,0,0,0));

    cvSetMouseCallback( "Select Points", on_mouse, &mouseParam );
    cvShowImage( "Select Points", img );

	cvWaitKey(0);
    cvDestroyWindow( "Select Points" );
	cvReleaseImage( &img );

	//ShowWindow( hWnd, SW_SHOW );
	scanner->Run();
	//ShowWindow( hWnd, SW_HIDE );

	img = cvLoadImage( "Image/000001.jpg");
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);
    cvNamedWindow( "Select Points", CV_WINDOW_AUTOSIZE );
    cvPutText(img,"Choose 4 corners on Horizontal Plane" , cvPoint(130,20),&font, cvScalar(0,0,0,0)); //center the text and write it with black color
	cvPutText(img,"Starting From Bottom Left(clockwise)" , cvPoint(130,40),&font, cvScalar(0,0,0,0));

    cvSetMouseCallback( "Select Points", on_mouse, &mouseParam );
    cvShowImage( "Select Points", img );

	cvWaitKey(0);
    cvDestroyWindow( "Select Points" );
	cvReleaseImage( &img );

	img = cvLoadImage( "Image/000001.jpg");
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,CV_AA);
    cvNamedWindow( "Select Points", CV_WINDOW_AUTOSIZE );
    cvPutText(img,"Choose 4 corners on Vertical Plane" , cvPoint(130,20),&font, cvScalar(0,0,0,0)); //center the text and write it with black color
	cvPutText(img,"Starting From Bottom Left(clockwise)" , cvPoint(130,40),&font, cvScalar(0,0,0,0));

    cvSetMouseCallback( "Select Points", on_mouse, &mouseParam );
    cvShowImage( "Select Points", img );

	cvWaitKey(0);
    cvDestroyWindow( "Select Points" );
	cvReleaseImage( &img );


	// TEST ZONE - DANGER!

	img = cvLoadImage( "Image/000001.jpg");

	//scanner->initializeVariables();
	scanner->computeExtrinsic(img);
	scanner->Reconstruct();
	scanner->WriteVRML();

//	cvFindCornerSubPix(grayImg, scanner.extPointsH, 4, Size(11,11), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
//
//	float grid_pointsV[4][3] = {{ 0, 0, 0} ,
//                                {0,-dY, 0} ,
//                                {dX, -dY, 0} ,
//                                {dX, 0, 0}
//                              };
//
//	float grid_pointsH[4][3] = {{ 0, 0, 0} ,
//                                {dX,0, 0} ,
//                                {0, -dY, 0} ,
//                                {dX, -dY, 0}
//                              };
//
//    CvMat X_gridV;
//	X_gridV= cvMat(4,3,CV_32FC1,grid_pointsV);
//
//	printCVmat(&X_gridV, 4,3);
//
//	CvMat X_gridH;
//	X_gridH = cvMat(4,3,CV_32FC1, grid_pointsH);
//	
//	printCVmat(&X_gridH, 4,3);
//
////    // create the points_on_plane matrix, which will be used for computing the extrinsic parameters
//    float points_on_plane [4][2]={{scanner.extPointsH[0].x, scanner.extPointsH[0].y} ,
//                                  {scanner.extPointsH[1].x, scanner.extPointsH[1].y} ,
//                                  {scanner.extPointsH[2].x, scanner.extPointsH[2].y} ,
//                                  {scanner.extPointsH[3].x, scanner.extPointsH[3].y}
//                                 };
//    CvMat plane_points;
//    plane_points= cvMat(4,2,CV_32FC1,points_on_plane);
//
//	printCVmat(&plane_points, 4,2);
//
//
//    float intrinsic_parameters [3][3]={{fc[0], 0, cc[0]} ,
//                                       {0, fc[1], cc[1]} ,
//                                       {0,    0 ,     1} ,
//                                      };
////    //Create the intrinsic matrix, as a CvMat* structure, for extrinsic computation
//    CvMat intrinsic;
//    intrinsic= cvMat(3,3,CV_32FC1,intrinsic_parameters);
//
//	printCVmat(&intrinsic, 3,3);
//
 //   float distortion_coeficients [5][1]={{kc[0]} ,
 //                                        {kc[1]} ,
 //                                        {kc[2]} ,
 //                                        {kc[3]} ,
 //                                        {kc[4]} ,
 //                                       };
 //   CvMat distort;
 //   distort= cvMat(5,1,CV_32FC1,distortion_coeficients);

	//MatrixXd m = cvmat2matrix(&distort);

	//cout << m;
//
//	printCVmat(&distort, 5,1);
//
//    // Compute the extrinsic parameters
//	CvMat* rotation_vector;
//	rotation_vector=cvCreateMat(1,3,CV_32FC1);
//	CvMat* translation_vector;
//	translation_vector =cvCreateMat(1,3,CV_32FC1);
//
//	cvFindExtrinsicCameraParams2(&X_gridH, &plane_points, &intrinsic, &distort, rotation_vector,translation_vector,0);
//
//	printCVmat(translation_vector, 1, 3);
//
//    CvMat* rotation_matrix;
//    rotation_matrix=cvCreateMat(3,3,CV_32FC1);
//    cvRodrigues2(rotation_vector,rotation_matrix,0);
//
//	printCVmat(rotation_matrix, 3, 3);



	//CopyFile(L"C:\\Documents and Settings\\ozan\\Belgelerim\\Visual Studio 2010\\Projects\\VisionProj\\VisionProj\\Model.wrl", L"C:\\3DScanner\\Model.wrl",false);
	//CopyFile(L"C:\\Documents and Settings\\ozan\\Belgelerim\\Visual Studio 2010\\Projects\\VisionProj\\Release\\VisionProj.exe", L"C:\\3DScanner\\VisionProj.exe",false);
	
	ShowWindow( hWnd, SW_SHOW );

	//system("pause");

	

    return 0;
}