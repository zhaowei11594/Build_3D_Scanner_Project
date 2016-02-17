#ifndef SCANNER_H
#define SCANNER_H

#include "Globals.h"
#include "Image.h"

class Scanner3D
{
public:

    // define constructor
	Scanner3D();

    //define variables
	vector<CvPoint> Points;
	CvPoint2D32f * extPointsH;
	CvPoint2D32f * extPointsV;

	Vector3d lowerLine;
	Vector3d middleLine;
	Vector3d upperLine;

	VectorXi vRows, hRows;
	VectorXi vCols, hCols;

	VectorXi recFrames;

	MatrixXd shadowValues;
	MatrixXd minValues;
	MatrixXd maxValues;

	MatrixXd vLineEnter;
	MatrixXd vLineLeave;
	MatrixXd hLineEnter;
	MatrixXd hLineLeave;

	MatrixXd shadowEnter;
	MatrixXd shadowLeave;

	MatrixXd vertices;
	MatrixXd colors;

	CvMat cameraMatrix;
	CvMat distCoeffs;
	CvMat gridPointsH;
	CvMat gridPointsV;
	CvMat* rotVectorH;
	CvMat* transVectorH;
	CvMat* rotMatrixH;
	CvMat* rotVectorV;
	CvMat* transVectorV;
	CvMat* rotMatrixV;


	vector<Vector3d> verticesTemp2;
	vector<Vector3d> colorTemp2;

    //define functions for 3d scanner use
	void Run();
	void EstimateShadowThresholds(bool readFromFile);
	void EstimateParametersOfShadowPlane();
	void EstimateShadowCrossingTimes();
	void initializeVariables();
	void computeExtrinsic(IplImage* img);
	void Reconstruct();
	bool WriteVRML();

	void readFrameIndex();
	void readShadowValues();
	void readMaxValues();
	void readMinValues();
};

#endif
