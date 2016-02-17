#ifndef GLOBALS_H
#define GLOBALS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cv.h>
#include "highgui.h"
#include "Eigen/Dense"
#include "Calibration.h"

using namespace Eigen;
using namespace std;
using namespace cv;

double round(double r);
bool isNan(double d);
void printCVmat(CvMat * mat, int rows, int cols);
void writeToFile(MatrixXd m, const char* name);

Vector3d fitLine(VectorXd X, VectorXd Y);
Vector4d fitPlane(VectorXd X, VectorXd Y, VectorXd Z);
Vector2d intersectLines(Vector3d& W1, Vector3d& W2);
Vector2d intersectLines(VectorXd W1, VectorXd W2);
Vector3d intersectLineWithPlane(Vector3d& q, Vector3d& v, Vector4d& w);
VectorXi initializeVector(int start, int end);

IplImage * getImage(int index);
IplImage * rgb2gray(IplImage * image);
MatrixXd imageToMatrix(IplImage * img);
IplImage * matrixToImage(MatrixXd mat);
IplImage * copyImage(IplImage * img);
IplImage * im2double(IplImage * image);
IplImage* Sub_Image(IplImage *image, CvRect roi);
VectorXd vectorToDouble(VectorXi vInt);
void extractGrid(CvPoint2D32f * cornerPts);
Vector2d Normalize_function(Vector2d& x);
Vector2d oulu_distortion(Vector2d& x_distort);
Vector3d Pixel2Ray (Vector2d& x);
VectorXd absValue(VectorXd v);
MatrixXd computeCollineation(Vector3d& a00, Vector3d& a10, Vector3d& a11, Vector3d& a01);
MatrixXd projectedGrid(Vector2d& P1, Vector2d& P2, Vector2d& P3, Vector2d& P4, int nx, int ny);
MatrixXd cvmat2matrix(CvMat * mat);
VectorXd cvmat2vect(CvMat * mat);

#endif
