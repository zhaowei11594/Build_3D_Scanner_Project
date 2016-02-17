#include "3DScanner.h"


Scanner3D::Scanner3D()
{
	//Horizontal points of the plane used for computing the extrinsic parameters
	extPointsH = new CvPoint2D32f[4];

    //Vertical points of the plane used for computing the extrinsic parameters
	extPointsV = new CvPoint2D32f[4];
}


void Scanner3D::initializeVariables()
{
	vRows = initializeVector(min(Points[3].y, Points[2].y),max(Points[3].y, Points[2].y));

	vCols = initializeVector(min(Points[3].x, Points[2].x),max(Points[3].x, Points[2].x));

	hRows = initializeVector(min(Points[5].y, Points[4].y),max(Points[5].y, Points[4].y));

	hCols = initializeVector(min(Points[5].x, Points[4].x),max(Points[5].x, Points[4].x));

	recFrames = initializeVector(frames_start, frames_end);

}


void Scanner3D::EstimateParametersOfShadowPlane()
{
	initializeVariables();

	IplImage * frame = getImage(1);

	int rows = frame->height;
	int cols = frame->width;

    //setting up variables for easiness of use in calculation
	VectorXd v(2);
	v << 0, cols;

	VectorXd ones(2);
	ones << 1, 1;

	VectorXd v2(2);
	v2 << 0.5, 0.5;

	VectorXd lower1 = v + v2;

	VectorXd lower2 = (rows*ones) + v2;

	VectorXd middlePt1(2);
	middlePt1 << Points[0].x, Points[1].x;

	VectorXd middlePt2(2);
	middlePt2 << Points[0].y, Points[1].y;

	VectorXd upper1 = v + v2;

	VectorXd upper2 = ones*0.5;

    //Calculate equations for lower/middle/upper boundaries.
	lowerLine  = fitLine(lower1, lower2);

	middleLine = fitLine(middlePt1, middlePt2);
	middleLine = -1 * middleLine;

	upperLine  = fitLine(upper1, upper2);

    // Estimate the shadow plane(s) for each frame.
	MatrixXd vLineEnterTemp(frames_end - frames_start + 1, 3); //vertical "entering" shadow
	MatrixXd vLineLeaveTemp(frames_end - frames_start + 1, 3); //vertical "leaving" shadow
	MatrixXd hLineEnterTemp(frames_end - frames_start + 1, 3); //horizontal "entering" shadow
	MatrixXd hLineLeaveTemp(frames_end - frames_start + 1, 3); //horizontal "leaving" shadow
	VectorXd vRowPosEnter(vRows.size()); //column of vertical "entering" shadow
	VectorXd vRowPosLeave(vRows.size()); //column of vertical "leaving" shadow
	VectorXd hRowPosEnter(hRows.size()); //column of horizontal "entering" shadow
	VectorXd hRowPosLeave(hRows.size()); //column of horizontal "leaving" shadow

	IplImage * image;

	for(int i=0; i<recFrames.size(); i++)
	{
	    // Extract the current frame and convert to grayscale).
		image = getImage(recFrames(i));
		image = rgb2gray(image);

		//Evaluating the vertical shadow line

		IplImage * subImg = Sub_Image(image, cvRect(vCols(0),vRows(0),vCols.size(), vRows.size()));

		MatrixXd matVImg = imageToMatrix(subImg);

		MatrixXd shadowVals = shadowValues.block(vRows(0), vCols(0), vRows.size(), vCols.size());

		MatrixXd vImg = matVImg - shadowVals;

		for(int j=1; j<vCols.size(); j++)
		{
			int * idx1 = new int[vRows.size()];
			int * idx2 = new int[vRows.size()];

			for(int k=0; k<vRows.size(); k++)
			{
				idx1[k] = (vImg(k,j) >= 0) & (vImg(k,j-1) < 0);
			}

			for(int k=0; k<vRows.size(); k++)
			{
				idx2[k] = (vImg(k,j) < 0) & (vImg(k,j-1) >= 0);
			}

			for(int k=0; k<vRows.size(); k++)
			{
				if(idx1[k]==1)
					vRowPosEnter(k) = j + (-vImg(k,j-1)) / (vImg(k,j)-vImg(k,j-1)) + vCols(0) - 1;
				if(idx2[k]==1)
					vRowPosLeave(k) = j + (-vImg(k,j-1)) / (vImg(k,j)-vImg(k,j-1)) + vCols(0) - 1;
			}
		}

		Vector3d line = fitLine(vRowPosEnter,vectorToDouble(vRows));

		vLineEnterTemp(i,0) = line(0);
		vLineEnterTemp(i,1) = line(1);
		vLineEnterTemp(i,2) = line(2);

		line = fitLine(vRowPosLeave,vectorToDouble(vRows));

		vLineLeaveTemp(i,0) = line(0);
		vLineLeaveTemp(i,1) = line(1);
		vLineLeaveTemp(i,2) = line(2);

		//Evaluating the horizontal shadow line

		subImg = Sub_Image(image, cvRect(hCols(0),hRows(0),hCols.size(), hRows.size()));

		matVImg = imageToMatrix(subImg);

		shadowVals = shadowValues.block(hRows(0), hCols(0), hRows.size(), hCols.size());

		MatrixXd hImg = matVImg - shadowVals;

		for(int j=1; j<hCols.size(); j++)
		{
			int * idx1 = new int[hRows.size()];
			int * idx2 = new int[hRows.size()];

			for(int k=0; k<hRows.size(); k++)
			{
				idx1[k] = (hImg(k,j) >= 0) & (hImg(k,j-1) < 0);
			}

			for(int k=0; k<hRows.size(); k++)
			{
				idx2[k] = (hImg(k,j) < 0) & (hImg(k,j-1) >= 0);
			}

			for(int k=0; k<hRows.size(); k++)
			{
				if(idx1[k]==1)
					hRowPosEnter(k) = j + (-hImg(k,j-1)) / (hImg(k,j)-hImg(k,j-1)) + hCols(0) - 1;
				if(idx2[k]==1)
					hRowPosLeave(k) = j + (-hImg(k,j-1)) / (hImg(k,j)-hImg(k,j-1)) + hCols(0) - 1;
			}
		}

		line = fitLine(hRowPosEnter,vectorToDouble(hRows));

		hLineEnterTemp(i,0) = line(0);
		hLineEnterTemp(i,1) = line(1);
		hLineEnterTemp(i,2) = line(2);

		line = fitLine(hRowPosLeave,vectorToDouble(hRows));

		hLineLeaveTemp(i,0) = line(0);
		hLineLeaveTemp(i,1) = line(1);
		hLineLeaveTemp(i,2) = line(2);


		// Draw shadow boundaries

		for(int x=0; x<vRowPosEnter.size(); x++)
		{
			Vector2d point(vRowPosEnter(x), vRows(x));

			CvPoint pt;

			pt.x = point(0); pt.y = point(1);

			cvLine(image, pt, pt, CV_RGB(255, 255, 255), 4);
		}

		for(int x=0; x<vRowPosLeave.size(); x++)
		{
			Vector2d point(vRowPosLeave(x), vRows(x));

			CvPoint pt;

			pt.x = point(0); pt.y = point(1);

			cvLine(image, pt, pt, CV_RGB(255, 255, 255), 4);
		}

		for(int x=0; x<hRowPosEnter.size(); x++)
		{
			Vector2d point(hRowPosEnter(x), hRows(x));

			CvPoint pt;

			pt.x = point(0); pt.y = point(1);

			cvLine(image, pt, pt, CV_RGB(255, 255, 255), 4);
		}

		for(int x=0; x<hRowPosLeave.size(); x++)
		{
			Vector2d point(hRowPosLeave(x), hRows(x));

			CvPoint pt;

			pt.x = point(0); pt.y = point(1);

			cvLine(image, pt, pt, CV_RGB(255, 255, 255), 4);
		}

		// Draw shadow lines

		Vector3d vEnter = vLineEnterTemp.row(i);
		Vector3d vLeave = vLineLeaveTemp.row(i);
		Vector3d hEnter = hLineEnterTemp.row(i);
		Vector3d hLeave = hLineLeaveTemp.row(i);

		Vector2d p1, p2;  CvPoint pt1, pt2;

		p1 = intersectLines(vEnter,middleLine);
		p2 = intersectLines(vEnter,upperLine);


		pt1.x = p1(0); pt1.y = p1(1); pt2.x = p2(0); pt2.y = p2(1);

		cvLine(image, pt1, pt2, CV_RGB(0,0,255), 1);

		p1 = intersectLines(vLeave,middleLine);
		p2 = intersectLines(vLeave,upperLine);


		pt1.x = p1(0); pt1.y = p1(1); pt2.x = p2(0); pt2.y = p2(1);

		cvLine(image, pt1, pt2, CV_RGB(0,255,0), 1);

		p1 = intersectLines(hEnter,lowerLine);
		p2 = intersectLines(hEnter,middleLine);


		pt1.x = p1(0); pt1.y = p1(1); pt2.x = p2(0); pt2.y = p2(1);

		cvLine(image, pt1, pt2, CV_RGB(0,0,255), 1);

		p1 = intersectLines(hLeave,lowerLine);
		p2 = intersectLines(hLeave,middleLine);


		pt1.x = p1(0); pt1.y = p1(1); pt2.x = p2(0); pt2.y = p2(1);

		cvLine(image, pt1, pt2, CV_RGB(0,255,0), 1);

		cvShowImage("Shadow Boundaries", image);

        waitKey(1);
	}

	vLineEnter = vLineEnterTemp;
	vLineLeave = vLineLeaveTemp;
	hLineEnter = hLineEnterTemp;
	hLineLeave = hLineLeaveTemp;

    cvDestroyWindow( "Shadow Boundaries" );
}

void Scanner3D::EstimateShadowThresholds(bool readFromFile)
{
    // Determine the minimum and maximum values observed in each pixel.

    // if the values aren't read from the file
	if(!readFromFile)
	{
		MatrixXd maxValue(ny, nx);
		MatrixXd minValue(ny, nx);

		for(int i=0; i<ny; i++)
		{
			for(int j=0; j<nx; j++)
			{
				maxValue(i,j) = -INF;
				minValue(i,j) = INF;
			}
		}

    //for each frame, compute the min and max value, in real time
		for(int i=0; i<num_frames; i++)
		{
            IplImage * frame = getImage(i+1);
			frame = rgb2gray(frame);

			MatrixXd frameMat = imageToMatrix(frame);

			for(int k=0; k<ny; k++)
				for(int j=0; j<nx; j++)
				{
					if( frameMat(k,j) < minValue(k,j) )
						minValue(k,j) = frameMat(k,j);
					if( frameMat(k,j) > maxValue(k,j) )
						maxValue(k,j) = frameMat(k,j);
				}
		}

		minValues = minValue;

		maxValues = maxValue;

		shadowValues = 0.5*(minValue + maxValue);
	}

	else  // read them directly from file
	{
		readShadowValues();
		readMinValues();
		readMaxValues();
	}
}

void Scanner3D::EstimateShadowCrossingTimes()
{
    //Determine the shadow crossing times for each pixel

	MatrixXd shadowEnterTemp(shadowValues.rows(), shadowValues.cols());
	MatrixXd shadowLeaveTemp(shadowValues.rows(), shadowValues.cols());

	for(int i=0; i<shadowValues.rows(); i++)
		for(int j=0; j<shadowValues.cols(); j++)
		{
			shadowEnterTemp(i,j) = _Nan._Double; //time shadow "enters" pixel
			shadowLeaveTemp(i,j) = _Nan._Double; //time shadow "leaves" pixel
		}

	IplImage * frame2 = getImage(recFrames(0));
	frame2 = rgb2gray(frame2);

	for(int i=1; i<recFrames.size(); i++)
	{
		IplImage * frame1 = copyImage(frame2);
		frame2 = getImage(recFrames(i));
		frame2 = rgb2gray(frame2);

		MatrixXd frame1mat = imageToMatrix(frame1);
		MatrixXd frame2mat = imageToMatrix(frame2);

		MatrixXi idx1(shadowValues.rows(), shadowValues.cols());
		MatrixXi idx2(shadowValues.rows(), shadowValues.cols());

		for(int j=0; j<idx1.rows(); j++)
			for(int k=0; k<idx1.cols(); k++)
			{
				idx1(j,k) = (frame1mat(j,k) >= shadowValues(j,k)) & (frame2mat(j,k) < shadowValues(j,k)) & isNan(shadowEnterTemp(j,k));
				idx2(j,k) = (frame1mat(j,k) < shadowValues(j,k)) & (frame2mat(j,k) >= shadowValues(j,k));
			}

		for(int j=0; j<idx1.rows(); j++)
			for(int k=0; k<idx1.cols(); k++)
			{
				if( idx1(j,k) == 1 )
					shadowEnterTemp(j,k) = (i) + (shadowValues(j,k) - frame1mat(j,k))/(frame2mat(j,k) - frame1mat(j,k));
				if( idx2(j,k) == 1 )
					shadowLeaveTemp(j,k) = (i) + (shadowValues(j,k)-frame1mat(j,k))/(frame2mat(j,k)-frame1mat(j,k));
			}
	}

	for(int i=0; i<shadowValues.rows(); i++)
		for(int j=0; j<shadowValues.cols(); j++)
		{
			if( maxValues(i,j) - minValues(i,j) < minContrast )  //min. contrast test
				shadowEnterTemp(i,j) = _Nan._Double;
			if( maxValues(i,j) - minValues(i,j) < minContrast )  //min. contrast test
				shadowLeaveTemp(i,j) = _Nan._Double;
		}

	shadowEnter = shadowEnterTemp;
	shadowLeave = shadowLeaveTemp;

}

void Scanner3D::readShadowValues()
{
	MatrixXd tempValues(ny, nx);

	string line;
	ifstream file("shadowValue.txt");

	if (file.is_open())
	{
		for(int i=0; i<ny; i++)
			for(int j=0; j<nx; j++)
			{
				if(!file.eof())
				{
					getline(file,line);

					double value = atof(line.c_str());

					tempValues(i,j) = value;
				}
			}

		file.close();
	}

	shadowValues = tempValues;
}


void Scanner3D::readFrameIndex()
{
	string line;
	ifstream file("frameIndex.txt");

	if (file.is_open())
	{
		if(!file.eof())
		{
			getline(file,line);

			int value = atoi(line.c_str());

			num_frames = value;
			cout << num_frames << endl;
			getline(file,line);

		    value = atoi(line.c_str());

			frames_start = value;
			cout << frames_start<< endl;
			getline(file,line);

			value = atoi(line.c_str());

			frames_end = value;
			cout << frames_end<< endl;
		}

		file.close();
	}
}

void Scanner3D::readMinValues()
{
	MatrixXd tempValues(ny, nx);

	string line;
	ifstream file("minValue.txt");

	if (file.is_open())
	{
		for(int i=0; i<ny; i++)
			for(int j=0; j<nx; j++)
			{
				if(!file.eof())
				{
					getline(file,line);

					double value = atof(line.c_str());

					tempValues(i,j) = value;
				}
			}

		file.close();
	}

	minValues = tempValues;
}

void Scanner3D::readMaxValues()
{
	MatrixXd tempValues(ny, nx);

	string line;
	ifstream file("maxValue.txt");

	if (file.is_open())
	{
		for(int i=0; i<ny; i++)
			for(int j=0; j<nx; j++)
			{
				if(!file.eof())
				{
					getline(file,line);

					double value = atof(line.c_str());

					tempValues(i,j) = value;
				}
			}

		file.close();
	}

	maxValues = tempValues;
}

void Scanner3D::Run()
{

	EstimateShadowThresholds(false);
	EstimateParametersOfShadowPlane();
	EstimateShadowCrossingTimes();
}

//Compute Extrinsic Parameters
void Scanner3D::computeExtrinsic(IplImage* img)
{
	IplImage * grayImg = rgb2gray(img);

	//Define the camera matrix

	float camMat [3][3]={{fc[0], 0, cc[0]} ,
                         {0, fc[1], cc[1]} ,
                         {0,    0 ,     1}};

    cameraMatrix = cvMat(3,3,CV_32FC1,camMat);

	cout << "Camera Matrix:" << endl;
	printCVmat(&cameraMatrix, 3,3);

    //Define distortion coefficients
	float dist_coeff [5][1]={{kc[0]}, {kc[1]}, {kc[2]}, {kc[3]}, {kc[4]} };

    distCoeffs = cvMat(5,1,CV_32FC1,dist_coeff);

	cout << "Distortion Coefficients:" << endl;
	printCVmat(&distCoeffs, 5,1);

    //Define grid points
	float grid_pointsV[4][3] = {{ 0, 0, 0} ,   //vertical
                                {0, dY, 0} ,
                                {dX, dY, 0} ,
                                {dX, 0, 0}
                              };

	float grid_pointsH[4][3] = {{ 0, 0, 0} ,  //horizontal
                                { 0, dY, 0} ,
                                { dX, dY, 0} ,
                                { dX, 0, 0}
                              };

	gridPointsV = cvMat(4,3,CV_32FC1,grid_pointsV);

	gridPointsH = cvMat(4,3,CV_32FC1,grid_pointsH);

	cout << "Grid Points Vertical:" << endl;
	printCVmat(&gridPointsV, 4,3);

	cout << "Grid Points Horizontal:" << endl;
	printCVmat(&gridPointsH, 4,3);

	for(int i=0; i<4; i++)
		cout << extPointsH[i].x << " " << extPointsH[i].y << endl;

	for(int i=0; i<4; i++)
		cout << extPointsV[i].x << " " << extPointsV[i].y << endl;

    //Refine selected corners to sub-pixel precision.
	cvFindCornerSubPix(grayImg, extPointsH, 4, Size(11,11), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	cvFindCornerSubPix(grayImg, extPointsV, 4, Size(11,11), Size(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	for(int i=0; i<4; i++)
		cout << extPointsH[i].x << " " << extPointsH[i].y << endl;

	for(int i=0; i<4; i++)
		cout << extPointsV[i].x << " " << extPointsV[i].y << endl;

	float cPtsH[4][2] = {   {extPointsH[0].x, extPointsH[0].y},
							{extPointsH[1].x, extPointsH[1].y},
							{extPointsH[2].x, extPointsH[2].y},
							{extPointsH[3].x, extPointsH[3].y}
                        };

	float cPtsV[4][2] = {   {extPointsV[0].x, extPointsV[0].y},
							{extPointsV[1].x, extPointsV[1].y},
							{extPointsV[2].x, extPointsV[2].y},
							{extPointsV[3].x, extPointsV[3].y}
                        };


	CvMat cornerPointsH;
	cornerPointsH = cvMat(4,2,CV_32FC1,cPtsH);

	cout << "Horizontal Corner Points:" << endl;
	printCVmat(&cornerPointsH, 4,2);

	CvMat cornerPointsV;
	cornerPointsV = cvMat(4,2,CV_32FC1,cPtsV);

	cout << "Vertical Corner Points:" << endl;
	printCVmat(&cornerPointsV, 4,2);

    //Create variables
	rotVectorH = cvCreateMat(1,3,CV_32FC1);

	transVectorH = cvCreateMat(1,3,CV_32FC1);

	rotMatrixH = cvCreateMat(3, 3, CV_32FC1);

	rotVectorV = cvCreateMat(1,3,CV_32FC1);

	transVectorV = cvCreateMat(1,3,CV_32FC1);

	rotMatrixV = cvCreateMat(3, 3, CV_32FC1);

    //Find the extrinsic parameters for the horizontal plane
	cvFindExtrinsicCameraParams2(&gridPointsH, &cornerPointsH, &cameraMatrix, &distCoeffs, rotVectorH, transVectorH, 0);

	cout << "Rotation Vector H: " << endl;
	printCVmat(rotVectorH, 1, 3);

	cout << "Translation Vector H: " << endl;
	printCVmat(transVectorH, 1, 3);

	cout << "Rotation Matrix H: " << endl;

	//Because cvFindExtrinsicCameraParams2 gives us only a rotation vector, we need to use cvRodrigues2 to obtain
	//the rotation matrix
	cvRodrigues2(rotVectorH,rotMatrixH,0);
	printCVmat(rotMatrixH, 3, 3);

    //Find the extrinsic parameters for the vertical plane
	cvFindExtrinsicCameraParams2(&gridPointsV, &cornerPointsV, &cameraMatrix, &distCoeffs, rotVectorV, transVectorV,0);

	cout << "Rotation Vector V: " << endl;
	printCVmat(rotVectorV, 1, 3);

	cout << "Translation Vector V: " << endl;
	printCVmat(transVectorV, 1, 3);

	cout << "Rotation Matrix V: " << endl;

	//Because cvFindExtrinsicCameraParams2 gives us only a rotation vector, we need to use cvRodrigues2 to obtain
	//the rotation matrix
	cvRodrigues2(rotVectorV,rotMatrixV,0);
	printCVmat(rotMatrixV, 3, 3);

}

// Reconstruct 3D Points
void Scanner3D::Reconstruct()
{
	MatrixXd RC_H = cvmat2matrix(rotMatrixH);
	MatrixXd RC_V = cvmat2matrix(rotMatrixV);
	VectorXd TC_H = cvmat2vect(transVectorH);
	VectorXd TC_V = cvmat2vect(transVectorV);

    // Estimate parameters of reference planes (using least-squares)

	MatrixXd X(3,4);

	int colNum = X.cols();
	int rowNum = X.rows();

	X << 0, dX, dX, 0,
		 0 ,0, dY, dY,
		 0 ,0 ,0 ,0;


	Vector4d Xrow1 = X.row(0);
	Vector4d Xrow2 = X.row(1);
	Vector4d Xrow3 = X.row(2);
	Vector4d hPlane;

	hPlane = fitPlane(Xrow1,Xrow2,Xrow3);

	VectorXd diffT_vh;

	diffT_vh = TC_V - TC_H;

	MatrixXd TempX(rowNum,colNum);

	for(int i = 0; i<colNum; i++)
		TempX.col(i) = diffT_vh;

	X = RC_H.transpose()*((RC_V * X) + TempX );

	Xrow1 = X.row(0);
	Xrow2 = X.row(1);
	Xrow3 = X.row(2);
	Vector4d vPlane;

	vPlane = fitPlane(Xrow1,Xrow2,Xrow3);

	vPlane = (-1)*vPlane;

    // Calculate camera center (in "horizontal" reference coordinate system)
	Vector3d C = ((-1)*RC_H.transpose()) * TC_H;

    // Determine implicit representation for the shadow planes
	MatrixXd shadowPlaneEnter(recFrames.rows(),4); //"entering" shadow plane
	MatrixXd shadowPlaneLeave(recFrames.rows(),4); //"leaving" shadow plane

	for(int i = 0; i<recFrames.size(); i++)
	{
		Vector3d n1_v,n2_v,p1_v,p2_v;

		VectorXd vLineEnterRow = vLineEnter.row(i);
		VectorXd hLineEnterRow = hLineEnter.row(i);

    //Determine true position of the "vertical" shadow boundary (entering)
		n1_v = RC_H.transpose() * Pixel2Ray(intersectLines(vLineEnterRow,middleLine));
		n2_v = RC_H.transpose() * Pixel2Ray(intersectLines(vLineEnterRow,upperLine));
        p1_v = intersectLineWithPlane(C,n1_v,vPlane);
        p2_v = intersectLineWithPlane(C,n2_v,vPlane);

		Vector3d n1_h,n2_h,p1_h,p2_h;
    //Determine true position of the "horizontal" shadow boundary (entering)
		n1_h = RC_H.transpose() * Pixel2Ray(intersectLines(hLineEnterRow,middleLine));
		n2_h = RC_H.transpose() * Pixel2Ray(intersectLines(hLineEnterRow,lowerLine));
		p1_h = intersectLineWithPlane(C,n1_h,hPlane);
	    p2_h = intersectLineWithPlane(C,n2_h,hPlane);


		Vector3d q_v,v_v,q_h,v_h;

    // Compute the "entering" shadow plane parameters
		q_v = p1_v;
		v_v = (p2_v-p1_v).normalized();
		q_h = p1_h;
		v_h = (p2_h-p1_h).normalized();

		shadowPlaneEnter.block(i,0,1,3) = v_v.cross(v_h).transpose();
		shadowPlaneEnter.block(i,0,1,3) = shadowPlaneEnter.block(i,0,1,3).normalized();
		VectorXd shadowEnterBlock = shadowPlaneEnter.block(i,0,1,3).transpose();

		double product = shadowEnterBlock.adjoint()*(q_v+q_h);

		shadowPlaneEnter(i,3) = 0.5 * product;

    // Determine true position of the "vertical" shadow boundary (leaving)
		n1_v = RC_H.transpose() * Pixel2Ray(intersectLines(vLineEnterRow,middleLine));
        n2_v = RC_H.transpose() * Pixel2Ray(intersectLines(vLineEnterRow,upperLine));
        p1_v = intersectLineWithPlane(C,n1_v,vPlane);
        p2_v = intersectLineWithPlane(C,n2_v,vPlane);

    // Determine true position of the "horizontal" shadow boundary (leaving)
		n1_h = RC_H.transpose() * Pixel2Ray(intersectLines(hLineEnterRow,middleLine));
		n2_h = RC_H.transpose() * Pixel2Ray(intersectLines(hLineEnterRow,lowerLine));
		p1_h = intersectLineWithPlane(C,n1_h,hPlane);
	    p2_h = intersectLineWithPlane(C,n2_h,hPlane);

		q_v = p1_v;
		v_v = (p2_v-p1_v).normalized();
		q_h = p1_h;
		v_h = (p2_h-p1_h).normalized();

    // Compute the "entering" shadow plane parameters
		shadowPlaneLeave.block(i,0,1,3) = v_v.cross(v_h).transpose();
		shadowPlaneLeave.block(i,0,1,3) = shadowPlaneLeave.block(i,0,1,3).normalized();
		VectorXd shadowLeaveBlock = shadowPlaneLeave.block(i,0,1,3).transpose();

		product = shadowLeaveBlock.adjoint()*(q_v+q_h);

		shadowPlaneLeave(i,3) = 0.5 * product;
	}

    //Reconstruct 3D points using intersection with shadow plane(s)
	IplImage * frameImg = getImage(1);

	RgbImage frame(frameImg);

	vector<int> rowIdx;
	vector<int> colIdx;

	for(int i=0; i<shadowValues.cols(); i++)
	{
		for(int j=0; j<shadowValues.rows(); j++)
		{
			if( !isNan(shadowEnter(j,i)) && !isNan(shadowLeave(j,i)) )
			{
				rowIdx.push_back(j);
				colIdx.push_back(i);
			}
		}
	}

	vector<int> rowIdxx;
	vector<int> colIdxx;

	for(int i=0; i<rowIdx.size(); i += dSample)
	{
		rowIdxx.push_back(rowIdx.at(i));
		colIdxx.push_back(colIdx.at(i));
	}

	int npts = rowIdxx.size();

	cout << "npts: "<< endl;
	cout << npts << endl << endl;

	MatrixXd verticesTemp(npts,3);

	MatrixXd colorsTemp = 0.65* MatrixXd::Ones(npts, 3);

	for(int i=0; i<npts; i++)
	{
	    //Obtain the camera ray for this pixel
		Vector3d n = RC_H.transpose() * Pixel2Ray(Vector2d(colIdxx.at(i),rowIdxx.at(i)));

		Vector4d wEnter;
		Vector3d pEnter;
		Vector4d wLeave;
		Vector3d pLeave;

        //Interpolate "entering" shadow plane parameters (using shadow time)
		if( !isNan(shadowEnter(rowIdxx.at(i),colIdxx.at(i))) )
		{
			double t = shadowEnter(rowIdxx.at(i),colIdxx.at(i));

			double t1 = floor(t);

			double t2 = t1 + 1;

			if(t2 <= recFrames.size())
			{
				double alpha = (t-t1)/(t2-t1);

				wEnter = (1-alpha)*shadowPlaneEnter.row(t1).transpose() + alpha*shadowPlaneEnter.row(t2).transpose();
				pEnter = intersectLineWithPlane(C,n,wEnter);
				verticesTemp.row(i) = pEnter.transpose();
			}
		}

        //Interpolate "leaving" shadow plane parameters (using shadow time)
		if( !isNan(shadowLeave(rowIdxx.at(i),colIdxx.at(i))) )
		{
			double t = shadowLeave(rowIdxx.at(i),colIdxx.at(i));

			double t1 = floor(t);

			double t2 = t1 + 1;

			if(t2 <= recFrames.size())
			{
				double alpha = (t-t1)/(t2-t1);

				wLeave = (1-alpha)*shadowPlaneEnter.row(t1).transpose() + alpha*shadowPlaneEnter.row(t2).transpose();
				pLeave = intersectLineWithPlane(C,n,wLeave);
				verticesTemp.row(i) = pLeave.transpose();
			}
		}

        // Average "entering" and "leaving" estimates (if both are available)
		if( !isNan(shadowLeave(rowIdxx.at(i),colIdxx.at(i))) && !isNan(shadowEnter(rowIdxx.at(i),colIdxx.at(i))) )
		{
			if( (pEnter-pLeave).norm() <= distReject )
				verticesTemp.row(i) = 0.5*(pEnter + pLeave).transpose();
			else  //If points do not agree, set coordinate to infinity. This will ensure that it is clipped by the bounding volume.
			{
				VectorXd ones(3);
				ones << 1,1,1;
				verticesTemp.row(i) = INF*ones.transpose();
			}
		}


		Vector3d colorLine;

        // Assign color per vertex (using source image)
		colorLine << (double)frame[rowIdxx.at(i)][colIdxx.at(i)].r/255.0, (double)frame[rowIdxx.at(i)][colIdxx.at(i)].g/255.0, (double)frame[rowIdxx.at(i)][colIdxx.at(i)].b/255.0;

		if(verticesTemp.row(i)(0)<10000 && verticesTemp.row(i)(1)<10000 && verticesTemp.row(i)(2)<10000)
		{
			Vector3d vertex = verticesTemp.row(i).transpose();
			Vector3d colorOfVertex = colorLine;

			verticesTemp2.push_back(vertex);
			colorTemp2.push_back(colorOfVertex);

		}

	}

}

// Write model to a VRML 1.0 file
bool Scanner3D::WriteVRML()
{
	std::string file_name = "Model.wrl";


	int i;
	using std::endl;

	// Test if mesh is valid
	if( (verticesTemp2.size()==0))
	{
		cout<<"No face or no vertex"<<endl;
		return false;
	}

	// Open file for writing
	std::ofstream file( file_name.c_str() );

	// Test if the file is open
	if( file.is_open() == false )
	{
		cout<<"crashed opening file"<<endl;
		return false;
	}

	//--
	// Write file Header
	//--

	// VRML 1.0 file header
	file<<"#VRML V1.0 ascii\n"<<endl;

	// Write vertex number (comment)
	file<<"# Vertices: "<<verticesTemp2.size()<<endl;

	// Begin description
	file<<"Separator {"<<endl;

	//--
	// Write point coordinates

	//--
	file<<"    Coordinate3 {"<<endl;
	file<<"        point ["<<endl;

	for( i=0; i<verticesTemp2.size(); i++ )
	{
		Vector3d singleRow = verticesTemp2.at(i);


		file<<"            "<<singleRow.row(0)<<" "<<singleRow.row(1)<<" "<<singleRow.row(2)<<","<<endl;
	}
	file<<"        ]"<<endl;
	file<<"    }"<<endl;

	file<<"    MaterialBinding {"<<endl;
	file<<"        value "<<"PER_VERTEX_INDEXED"<<endl;
	file<<"    }"<<endl;
	// Color
	file<<"    Material {"<<endl;
	file<<"        diffuseColor ["<<endl;
	for( i=0; i<colorTemp2.size(); i++ )
	{
		Vector3d singleRow = colorTemp2.at(i);

		file<<"            "<<singleRow.row(0)<<" "<<singleRow.row(1)<<" "<<singleRow.row(2)<<","<<endl;
	}
	file<<"        ]"<<endl;
	file<<"    }"<<endl;
	file<<"}"<<endl;



	// Close file
	file.close();

	/*CopyFile(L"C:\\Documents and Settings\\ozan\\Belgelerim\\Visual Studio 2010\\Projects\\VisionProj\\VisionProj\\Model.wrl", L"C:\\3DScanner",false);*/

	return true;

}



