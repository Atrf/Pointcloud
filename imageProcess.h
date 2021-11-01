#ifndef  __IMAGEPROCESS__
#define __IMAGEPROCESS__
#pragma once
#include <iostream>


#include <array>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


using namespace cv;
using namespace std;
class imageProcess
{
private:
	int iLowH = 3;
	int iHighH = 36;
	int iLowS = 87;
	int iHighS = 46;
	int iLowV = 98;
	int iHighV = 255;
	//int iLowH = 0;
	//int iHighH = 10;
	//int iLowS = 38;
	//int iHighS = 149;
	//int iLowV = 113;
	//int iHighV = 245;

public:
	imageProcess();
	int processImageGetGreenRegion(Mat& img);
	vector<vector<Point>> rectPoint;
};

#endif // __IMAGEPROCESS__
