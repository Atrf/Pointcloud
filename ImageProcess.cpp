#include "imageProcess.h"


imageProcess::imageProcess()
{
    array<int, 4> region = { 0 , 0 , 0, 0 };
}

int imageProcess::processImageGetGreenRegion(Mat& img)
{
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(img, imgHSV, COLOR_BGR2HSV);//转换图像的颜色，彩色 灰度，HSV 等等
    split(imgHSV, hsvSplit);//split函数的功能是通道分离
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);
    //imshow("imgHsV", imgHSV);
    //waitKey(0);
    Mat imgThresholded;
    /*namedWindow("TrackBars", (640 ,200));
    createTrackbar("iLowH", "TrackBars", &iLowH, 179);
    createTrackbar("iLowS", "TrackBars", &iLowS, 255);
    createTrackbar("iLowV", "TrackBars", &iLowV, 255);
    createTrackbar("iHighH", "TrackBars", &iHighH, 179);
    createTrackbar("iHighS", "TrackBars", &iHighS, 255);
    createTrackbar("iHighV", "TrackBars", &iHighV, 255);
    cout << iLowH << " " << iHighH << endl;*/
    /*while (true) {*/
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
 /*       imshow("img1", imgThresholded);
        waitKey(1);
    }*/
    //开操作 (去除一些噪点)
    Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    Mat element1 = getStructuringElement(MORPH_RECT, Size(21, 21));
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element1);


    vector<vector<Point> >contours;
    vector<Vec4i> hierarchy;
    findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    rectPoint.clear();
    int s = contours.size();
    cout << "s=" << s << endl;
    rectPoint.resize(s);
    for (int i = 0; i < s; i++)
    {
        //获得矩形外包围框
        Rect r = boundingRect(Mat(contours[i]));
        //RotatedRect r = minAreaRect(Mat(contours[i]));
        //cout << "contours" << i << "height=" << r.height << "width =" << r.width << "rate =" << ((float)r.width / r.height) << endl;
        //cout << "r.x = " << r.x << "  r.y  = " << r.y << "rate =" << ((float)r.width / r.height) << " area = " << ((float)r.width * r.height) << endl;
        Point upLeft, downRight;
        upLeft.x = r.x;
        upLeft.y = r.y;
        downRight.x = r.x + r.width;
        downRight.y = r.y + r.height;
        //cout << "x="<<upLeft.x << "y= " << upLeft.y << endl;
        vector<Point> vecPoint;
        vecPoint.resize(2);
        vecPoint[0] = upLeft;
        vecPoint[1] = downRight;
        rectPoint[i] = vecPoint;
    }

    return 1;
}
