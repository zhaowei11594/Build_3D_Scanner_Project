#ifndef IMAGE_H
#define IMAGE_H

#include <cv.h>
#include "highgui.h"

using namespace cv;

template <class T> class Image
{
private:
	IplImage *imgp;
public:
	Image(IplImage *img = 0) {imgp = img;}
	~Image() {imgp = 0;}
	void operator = (IplImage *img){imgp = img;}
	inline T* operator[](const int rowIndx){
		return ((T*)(imgp->imageData + rowIndx *imgp ->widthStep));}
};

typedef struct{
	unsigned char b, g, r;
}RgbPixel;

typedef struct{
	float b, g, r;
}RgbPixelFloat;

typedef Image<RgbPixel>      RgbImage;
typedef Image<RgbPixelFloat> RgbImageFloat;
typedef Image<unsigned char> BwImage;
typedef Image<float>         BwImageFloat;

#endif