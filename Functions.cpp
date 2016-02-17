#include "Globals.h"

//Useful functions through out the program

double round(double r)
{
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

void printCVmat(CvMat * mat, int rows, int cols)
{
	for (int i=0;i<rows;i++)
    {
        for (int j=0;j<cols;j++)
        {
            cout << mat->data.fl[i*cols+j] << " ";
        }

		cout << endl;
    }

	cout << endl;
}

bool isNan(double d)
{
    return (d != d);
}

void writeToFile(MatrixXd m, const char* name)
{
	string line;
	ofstream file(name);

	for(int i=0; i<m.rows(); i++)
	{
		for(int j=0; j<m.cols(); j++)
		{
			file << m(i,j) << " \t";
		}
		file << endl;
	}

	file.close();
}

IplImage * getImage(int index)
{
	char str1[15];
	char str2[4];
	char *str3 = ".jpg";

	if (index<10)
	{
		strcpy_s (str1,"00000");
		strcpy_s (str2,_itoa(index,str2,10));
		strncat (str1, str2, 7);
	}

	else if(9<index && index<100)
	{
		strcpy_s (str1,"0000");
		strcpy_s (str2,_itoa(index,str2,10));
		strncat (str1, str2, 7);
	}
	else
	{
		strcpy_s (str1,"000");
		strcpy_s (str2,_itoa(index,str2,10));
		strncat (str1, str2, 7);
	}

	char dir[20];
	strcpy_s(dir, "Image/");

	strncat (str1, str3, 7);
	strncat (dir, str1, 12);
	//puts();

	IplImage* frame = 0;
	frame = cvLoadImage(dir);

	return frame;
}

IplImage * rgb2gray(IplImage * image)
{
	IplImage* GrayFrame = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 1);
	cvCvtColor(image,GrayFrame,CV_RGB2GRAY );

	return GrayFrame;
}

IplImage * im2double(IplImage * image)
{
	IplImage * result = cvCreateImage(cvSize(image->width,image->height), image->depth, image->nChannels);

	for(int x=0;x<image->width;x++)
    {
        for(int y=0;y<image->height;y++)
        {
			double pixelValue = cvGetReal2D(image, y, x) / 255;

			//cout << pixelValue << endl;

			cvSetReal2D(result, y, x, pixelValue);
		}
	}

	return result;
}

Vector3d fitLine(VectorXd X, VectorXd Y)
{
	double meanX = X.mean();
	double meanY = Y.mean();

	MatrixXd U(X.size(),2);

	for(int i=0; i<X.size(); i++)
	{
		U(i,0) = (X(i) - meanX);
		U(i,1) = (Y(i) - meanY);
	}

	SelfAdjointEigenSolver<MatrixXd> eigensolver(U.transpose() * U);

	VectorXd eigValues= eigensolver.eigenvalues();

	//cout << U << endl << endl;

	//cout << eigValues << endl << endl;

	double minEigenValue = 999999999;
	int minIndex;

	for(int i=0; i<eigValues.size(); i++)
	{
		if(eigValues(i) < minEigenValue)
		{
			minEigenValue = eigValues(i);
			minIndex = i;
		}
	}

	MatrixXd eigVectors = eigensolver.eigenvectors();

	//cout << eigVectors << endl << endl;

	VectorXd minEigenVector = eigVectors.col(minIndex);

	//cout << minEigenVector << endl << endl;

	double a = minEigenVector(0);
	double b = minEigenVector(1);

	double d = (a * meanX) + (b * meanY);

	Vector3d dVect;

	dVect << a, b, d;

	//cout << dVect << endl << endl;

	return dVect;
}

Vector2d intersectLines(Vector3d& W1, Vector3d& W2)
{
	Matrix2d A;

	A << W1(0), W1(1), W2(0), W2(1);

	Vector2d b(W1(2), W2(2));

	Vector2d p = A.inverse() * b;

	//cout << p << endl << endl;

	return p;
}

Vector2d intersectLines(VectorXd W1, VectorXd W2)
{
	Matrix2d A;

	A << W1(0), W1(1), W2(0), W2(1);

	Vector2d b(W1(2), W2(2));

	Vector2d p = A.inverse() * b;

	//cout << p << endl << endl;

	return p;
}

VectorXi initializeVector(int start, int end)
{
	VectorXi tempVect(end - start + 1);

	for(int i=start; i<=end; i++)
		tempVect(i - start) = i;

	return tempVect;
}

IplImage * copyImage(IplImage * img)
{
	IplImage * result = cvCreateImage( cvSize(img->width, img->height), img->depth, img->nChannels );

	cvCopy( img, result, NULL);

	return result;
}

MatrixXd imageToMatrix(IplImage * img)
{
	MatrixXd mat(img->height, img->width);

	for(int i=0; i<img->height; i++)
		for(int j=0; j<img->width; j++)
		{
			CvScalar s = cvGet2D(img, i,j);
			mat(i,j) = s.val[0];
		}

	return mat;
}

MatrixXd cvmat2matrix(CvMat * mat)
{
	MatrixXd matrix(mat->rows, mat->cols);

	for (int i=0;i<mat->rows;i++)
    {
        for (int j=0;j<mat->cols;j++)
        {
            matrix(i,j) = mat->data.fl[i*mat->cols+j];
        }
    }

	return matrix;
}

VectorXd cvmat2vect(CvMat * mat)
{
	VectorXd vect(mat->rows*mat->cols);

	for (int i=0;i<mat->rows;i++)
    {
        for (int j=0;j<mat->cols;j++)
        {
            vect(j) = mat->data.fl[i*mat->cols+j];
        }
    }

	return vect;
}

IplImage * matrixToImage(MatrixXd mat)
{
	IplImage * result = cvCreateImage(cvSize(mat.cols(), mat.rows()), IPL_DEPTH_8U, 1);

	for(int y=0; y<mat.rows(); y++)
    {
        for(int x=0; x<mat.cols(); x++)
        {
			cvSetReal2D(result, y, x, mat(y,x));
		}
	}

	return result;
}

VectorXd vectorToDouble(VectorXi vInt)
{
	VectorXd vDouble(vInt.size());

	for(int i=0; i<vInt.size(); i++)
		vDouble(i) = (double)vInt(i);

	return vDouble;
}

IplImage * Sub_Image(IplImage *image, CvRect roi)
{
	IplImage *result;
	// set ROI, you may use following two funs:
	//cvSetImageROI( image, cvRect( 0, 0, image->width, image->height ));

	cvSetImageROI(image,roi);
	// sub-image
	result = cvCreateImage( cvGetSize(image), image->depth, image->nChannels );
	cvCopy(image,result);
	cvResetImageROI(image); // release image ROI

	return result;
}

Vector4d fitPlane(VectorXd X, VectorXd Y, VectorXd Z)
{
	double meanX = X.mean();
	double meanY = Y.mean();
	double meanZ = Z.mean();

	MatrixXd U(X.size(),3);

	for(int i=0; i<X.size(); i++)
	{
		U(i,0) = (X(i) - meanX);
		U(i,1) = (Y(i) - meanY);
		U(i,2) = (Z(i) - meanZ);
	}

	SelfAdjointEigenSolver<MatrixXd> eigensolver(U.transpose() * U);

	VectorXd eigValues= eigensolver.eigenvalues();

	double minEigenValue = 999999999;
	int minIndex;

	for(int i=0; i<eigValues.size(); i++)
	{
		if(eigValues(i) < minEigenValue)
		{
			minEigenValue = eigValues(i);
			minIndex = i;
		}
	}

	MatrixXd eigVectors = eigensolver.eigenvectors();

	//cout << eigVectors << endl << endl;

	VectorXd minEigenVector = eigVectors.col(minIndex);

	//cout << minEigenVector << endl << endl;

	double a = minEigenVector(0);
	double b = minEigenVector(1);
	double c = minEigenVector(2);

	double d = (a * meanX) + (b * meanY) + (c * meanZ);

	Vector4d dVect;

	dVect << a, b, c, d;

	//cout << dVect << endl << endl;

	return dVect;
}

Vector3d intersectLineWithPlane(Vector3d& q, Vector3d& v, Vector4d& w){

    Vector3d p;
    float depth;
	// Evaluate inner products.
	float n_dot_q = 0, n_dot_v = 0;
	for(int i=0; i<3; i++){
		n_dot_q += w[i]*q[i];
		n_dot_v += w[i]*v[i];
	}

	// Evaluate point of intersection P.
	depth = (w[3]-n_dot_q)/n_dot_v;
	for(int i=0; i<3; i++)
		p[i] = q[i] + depth*v[i];

    return p;
}

Vector2d Normalize_function(Vector2d& x)
{
    Vector2d x_distort;
    Vector2d xn;

    float distortion_coeficients [5][1]={{kc[0]} ,
                                         {kc[1]} ,
                                         {kc[2]} ,
                                         {kc[3]} ,
                                         {kc[4]} ,
                                        };
    CvMat distort;
    distort= cvMat(5,1,CV_32FC1,distortion_coeficients);

    // First: Subtract principal point, and divide by the focal length:
    for (int i=0;i<2;i++)
    {
        x_distort[i]= (x[i] - cc[i])/fc[i];
    }
    // Second: undo skew
    x_distort[0] = x_distort[0] - alpha_c * x_distort[1];

    double norm_value= cvNorm(&distort,0,CV_L2);

	if (x_distort.norm() != 0)
    {
        xn=oulu_distortion(x_distort);
    }
    else
        xn=x_distort;

    return xn;
}

Vector2d oulu_distortion(Vector2d& xd)
{
    double k1,k2,k3,p1,p2;
    Vector2d denominator;
    Vector2d x;
    Vector2d delta_x;

    k1=kc[0];
    k2=kc[1];
    k3=kc[4];
    p1=kc[2];
    p2=kc[3];

    for (int i=0;i<2;i++)
    {
        x[i]=xd[i];         //initial guess
    }

    for (int kk=0; kk<20;kk++)
    {
        double r_2 = pow(x[0],2) + pow(x[1],2);
        double k_radial = 1 + k1 * r_2 + k2 * pow(r_2,2) + k3 * pow(r_2,3);
        delta_x[0] = 2*p1*x[0]*x[1] + p2*(r_2 + 2*pow(x[0],2));
        delta_x[1] = p1 * (r_2 + 2*pow(x[1],2))+2*p2*x[0]*x[1];

        denominator[0] = 1*k_radial;
        denominator[1] = 1*k_radial;

        x[0]=xd[0]-delta_x[0] / denominator[0];
        x[1]=xd[1]-delta_x[1] / denominator[1];
    }
    return x;
}

Vector3d Pixel2Ray (Vector2d& x)
{
    Vector2d v_normalized;
    double sumation(0);
    Vector3d repeat_matrix;
    Vector3d v;

    v_normalized=Normalize_function(x);
    v[0] = v_normalized[0];
    v[1] = v_normalized[1];
    v[2] = 1;

    for (int i=0;i<3;i++)
    {
        sumation= sumation + pow(v[i],2);
    }

    double square_root=sqrt(sumation);

    //  This repeat_matrix would be the equivalent of repmat in matlab
    for (int i=0;i<3;i++)
    {
        repeat_matrix[i] = square_root;
    }

    for (int j=0;j<3;j++)
    {
        v[j] = v[j] / repeat_matrix[j];
    }

    return v;
}

MatrixXd projectedGrid(Vector2d& P1, Vector2d& P2, Vector2d& P3, Vector2d& P4, int nx, int ny)
{
	Vector3d a00(P1(0), P1(1), 1);
	Vector3d a10(P2(0), P2(1), 1);
	Vector3d a11(P3(0), P3(1), 1);
	Vector3d a01(P4(0), P4(1), 1);

	MatrixXd H = computeCollineation(a00, a10, a11, a01);

	MatrixXd pts(4,3);

	pts << 0,0,1,
			1,0,1,
			0,1,1,
			1,1,1;

	MatrixXd XX = H*pts.transpose();

	MatrixXd XXX(XX.rows()-1, XX.cols());

	for(int i=0; i<XXX.rows(); i++)
	{
		for(int j=0; j<XXX.cols(); j++)
		{
			XXX(i,j) = XX(i,j) / XX(2,j);
		}
	}

	return XXX;
}

VectorXd absValue(VectorXd v)
{
	VectorXd result(v.size());

	for(int i=0; i<v.size(); i++)
		result(i) = abs(v(i));

	return result;
}

MatrixXd computeCollineation(Vector3d& a00, Vector3d& a10, Vector3d& a11, Vector3d& a01)
{
	VectorXd ax(4);
	ax << a00(0), a10(0), a11(0), a10(0);
	VectorXd ay(4);
	ay << a00(1), a10(1), a11(1), a10(1);

	double mxx = ax.mean();
	double myy = ay.mean();

	ax = ax - mxx*Vector4d::Ones();
	ay = ay - myy*Vector4d::Ones();

	double scxx = absValue(ax).mean();
	double scyy = absValue(ay).mean();

	MatrixXd Hnorm(3,3);

	Hnorm << 1/scxx, 0, (-mxx/scxx),
			0, 1/scyy, -myy/scyy,
			0, 0, 1;

	MatrixXd invHnorm(3,3);

	invHnorm << scxx, 0, mxx,
				0, scyy, myy,
				0, 0, 1;

	Vector3d a00n = Hnorm * a00;
	Vector3d a10n = Hnorm * a10;
	Vector3d a11n = Hnorm * a11;
	Vector3d a01n = Hnorm * a01;

	Vector3d r = a00n.cross(a10n);

	Vector3d V1n = a00n.cross(a10n).cross(a01n.cross(a11n));
	Vector3d V2n = a00n.cross(a01n).cross(a10n.cross(a11n));

	MatrixXd V1 = invHnorm*V1n;
	MatrixXd V2 = invHnorm*V2n;

	V1n.normalize();
	V2n.normalize();

	double alpha_x = (a10n(1)*a00n(0) - a10n(0)*a00n(1))/(V1n(1)*a10n(0)-V1n(0)*a10n(1));
	double alpha_y = (a01n(1)*a00n(0) - a01n(0)*a00n(1))/(V2n(1)*a01n(0)-V2n(0)*a01n(1));

	MatrixXd Hrem(3,3);

	Hrem.col(0) = alpha_x*V1n;
	Hrem.col(1) = alpha_y*V2n;
	Hrem.col(2) = a00n;

	MatrixXd H = invHnorm*Hrem;

	return H;
}
