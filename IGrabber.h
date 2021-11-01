
#include <iostream>
#include "stdafx.h"  
#include <fstream>
#include<string>
#include <time.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/convolution.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include<opencv2/opencv.hpp>

#include "ImageRenderer.h" 
using namespace std;
using namespace cv;

struct vp {
	int x;
	int y;
};

class IGrabber
{
public:
	IGrabber()
	{
		//cout << "IGrabber Normal" << endl;
	}
	~IGrabber();
	bool SensorInit();
	bool GetImg(bool IsSaveImg, bool IsShowImg);
	void SaveImg(string command,string savepath);
	void deletedepth()
	{
		delete[] depthArray;
	}
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(int start_x, int start_y, int end_x, int end_y, string savepath);*/
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertDepthToPointXYZ();
	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthRegionToPointXYZ(const UINT16* depthBuffer, cv::Mat* colorimg);

	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZGuanFang(UINT16* depthBuffer);
	//将识别区域的深度图像转换为点云
	cv::Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// 转换depth图像到cv::Mat  
	cv::Mat  ConvertDepthMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// 转换16bit depth图像
	cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight);// 转换color图像到cv::Mat 
	UINT16* GetDepthbuffer();
	bool isMappingMatrixempty();
	UINT16* pBuffer_depth = NULL;					//depth buffer
	UINT16* depthArray = NULL;
	string getTime();
	cv::Mat depthImg_show;// = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);//原始UINT16 深度图像不适合用来显示，所以需要砍成8位的就可以了，但是显示出来也不是非常好，最好能用原始16位图像颜色编码，凑合着看了
	cv::Mat depthImg;// = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//the depth image
	cv::Mat colorImg;// = cv::Mat::zeros(color_height, color_widht, CV_8UC3);//the color image
	//cv::Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//原始UINT16 深度图像不适合用来显示，所以需要砍成8位的就可以了，但是显示出来也不是非常好，最好能用原始16位图像颜色编码，凑合着看了  
	//cv::Mat depthImg = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//the depth image  
	//cv::Mat colorImg = cv::Mat::zeros(color_height, color_widht, CV_8UC3);//the color image 
	//cv::Mat colorImg = cv::imread("D:\\Desktop\\photo\\11.jpg");

private:
	int depth_width = 512; //depth图像的大小  
	int depth_height = 424;
	int color_widht = 1920; //color图像的大小 
	int color_height = 1080;
	std::vector<ColorSpacePoint> MappingMatrix;
	HRESULT hr=S_OK;//表示函数执行状态
	IKinectSensor* m_pKinectSensor = NULL;			// Current Kinect    
	IDepthFrameReader* m_pDepthFrameReader = NULL;	// Depth reader深度帧的阅读器。
	IDepthFrameSource* pDepthFrameSource = NULL;	//depth frame source	 
	IColorFrameReader* m_pColorFrameReader = NULL;	// Color reader 
	IColorFrameSource* pColorFrameSource = NULL;	//color frame source
	ICoordinateMapper* m_pMapper = NULL;			// coordinate Mapper
	//RGBQUAD* m_pColorRGBX = new RGBQUAD[color_widht * color_height];
};
#pragma once
