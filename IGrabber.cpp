#include "IGrabber.h"
#include <imgcodecs.hpp>
#include <highgui.hpp>


//
//IGrabber::IGrabber()
//{
//	cout << "Normal" << endl;
//}

IGrabber::~IGrabber()
{
	//SafeRelease(m_pKinectSensor);
	//SafeRelease(m_pDepthFrameReader);
	//SafeRelease(pDepthFrameSource);
	//SafeRelease(m_pColorFrameReader);
	//SafeRelease(pColorFrameSource);
	//SafeRelease(m_pMapper);
	//SafeRelease(pBuffer_depth);
}

bool IGrabber::SensorInit()
{
	int colorWidth = 0;
	int colorHeight = 0;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);//获取感应器
	if (FAILED(hr))
	{
		cout << "FUCK! Can not find the Kinect!" << endl;
		return false;
	}//如果失败
	//在Kinect for windows SDK2.0中，获取并处理数据源接口步骤如下：Sensor->Source->Reader->Frame->Data
	hr = m_pKinectSensor->Open();//打开感应器Starts streaming data from the Kinect using a specified access mode.
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::Open()");
	}
	// Retrieved Coordinate Mapper
	hr = m_pKinectSensor->get_CoordinateMapper(&m_pMapper);//Returns S_OK if successful; otherwise, returns a failure code.;Gets the coordinate mapper.
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
	}

	// Retrieved Color Frame Source
	hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);//Gets the color source.
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_ColorFrameSource()");
	}
	hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	// Retrieved Depth Frame Source
	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
	}
	hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	// Retrieved Infrared Frame Source
	IInfraredFrameSource* infraredSource;//表示来自KinectSensor的红外帧源。
	hr = m_pKinectSensor->get_InfraredFrameSource(&infraredSource);
	if (FAILED(hr)) {
		throw std::exception("Exception : IKinectSensor::get_InfraredFrameSource()");
	}

	// Retrieved Color Frame Size  检索颜色帧大小
	IFrameDescription* colorDescription;//Represents the properties of an image frame from the KinectSensor.表示来自KinectSensor的图像帧的属性
	hr = pColorFrameSource->get_FrameDescription(&colorDescription);//Gets the description of the color frames.
	if (FAILED(hr)) {
		throw std::exception("Exception : IColorFrameSource::get_FrameDescription()");
	}

	hr = colorDescription->get_Width(&colorWidth); // 1920
	if (FAILED(hr)) {
		throw std::exception("Exception : IFrameDescription::get_Width()");
	}

	hr = colorDescription->get_Height(&colorHeight); // 1080
	if (FAILED(hr)) {
		throw std::exception("Exception : IFrameDescription::get_Height()");
	}

	//depth reader  
	if (!m_pDepthFrameReader)
	{
		cout << " Can not find the m_pDepthFrameReader!" << endl;
		cv::waitKey(0);//waitKey()函数的功能是不断刷新图像，频率为delay，单位是ms，返回值为当前键盘按下的值，没有按键时返回-1.参数为0表示会一直停顿在当前图像上
		exit(0);//exit（0）：正常运行程序并退出程序；exit（1）：非正常运行导致退出程序；return（）：返回函数，若在主函数中，则会退出函数并返回一值。
	}
	//color reader  
	if (!m_pColorFrameReader)
	{
		cout << "Can not find the m_pColorFrameReader!" << endl;
		cv::waitKey(0);
		exit(0);
	}

	SafeRelease(colorDescription);//This function releases the pointer and sets it equal to NULL.
	SafeRelease(infraredSource);
	return true;
}

bool IGrabber::GetImg(bool IsSaveImg, bool IsShowImg)
{
	
	UINT nBufferSize_depth = 0;//uint就是一个无符号的int,即unsigned int
	UINT nBufferSize_coloar = 0;
	RGBQUAD* pBuffer_color = NULL;
	UINT16* pBuffer_depth = NULL;
	depthArray = new UINT16[512 * 424]; //保存当前帧的深度数据，用于生成点云
	char key = 0;
	while (true) {
		IDepthFrame* pDepthFrame = nullptr;
		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hr))
		{
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxReliableDistance = 0;
			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			}
			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
				hr = pDepthFrame->CopyFrameDataToArray(nBufferSize_depth, reinterpret_cast<UINT16*>(depthArray));
				if (IsSaveImg || IsShowImg) {
					depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
					depthImg = ConvertDepthMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
				}
			}
		}
		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxDistance = 0;
			UINT nBufferSize = 0;
			UINT16* pBuffer = NULL;
			MappingMatrix.resize(512 * 424);

			hr = pDepthFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			}
			if (SUCCEEDED(hr)) {
				hr = pFrameDescription->get_Width(&nWidth);
			}
			if (SUCCEEDED(hr)) {
				hr = pFrameDescription->get_Height(&nHeight);
			}
			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}
			if (SUCCEEDED(hr)) {
				// In order to see the full range of depth (including the less reliable far field depth)
				// we are setting nDepthMaxDistance to the extreme potential depth threshold
				nDepthMaxDistance = USHRT_MAX;

				// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
				//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
			}
			if (SUCCEEDED(hr)) {
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			}
			if (SUCCEEDED(hr)) {
				ICoordinateMapper* pCoordinateMapper;
				m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
				pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, pBuffer, 512 * 424, &MappingMatrix[0]);
				//
				//pCoordinateMapper->MapDepthPointToCameraSpace();
			}
			SafeRelease(pFrameDescription);
		}
		SafeRelease(pDepthFrame);
		//for color		
		IColorFrame* pColorFrame = NULL;
		hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
		ColorImageFormat imageFormat = ColorImageFormat_None;
		if (SUCCEEDED(hr))
		{
			ColorImageFormat imageFormat = ColorImageFormat_None;
			if (SUCCEEDED(hr)) {
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}
			RGBQUAD* m_pColorRGBX = new RGBQUAD[color_widht * color_height];
			if (SUCCEEDED(hr)) {
				if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧  
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_coloar, reinterpret_cast<BYTE**>(&pBuffer_color));
				}
				else if (m_pColorRGBX)
				{
					pBuffer_color = m_pColorRGBX;
					nBufferSize_coloar = color_widht * color_height * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_coloar, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
				}

				else
				{
					hr = E_FAIL;
				}
				colorImg = ConvertMat(pBuffer_color, color_widht, color_height);
			}
			SafeRelease(pColorFrame);
			delete[] m_pColorRGBX;
			//m_pColorRGBX = NULL;
		}
		if (colorImg.empty() || depthImg.empty() || depthImg_show.empty())
			continue;
		if (!colorImg.empty() && !depthImg.empty() && !depthImg_show.empty())
			break;
		}
	if (IsShowImg) {
		namedWindow("depth", 0);
		cv::imshow("depth", depthImg_show);
		//cv::imshow("depth", depthImg);
		namedWindow("color", 0);
		cv::imshow("color", colorImg);
	}
	return true;
}


void IGrabber::SaveImg(string command,string savepath)
{
	//
	//获取时间戳
	string currtime = getTime();
	//定义存储路径
	savepath.append("\\");
	//存储3种图片
	cv::imwrite(savepath + currtime + "_gray.png", depthImg_show);
	cv::imwrite(savepath + currtime + "_gt.png", depthImg);
	cv::imwrite(savepath + currtime + "_rgb.bmp", colorImg);
	//存储映射关系
	char* g_buffer = new char[512 * 424 * 21];
	//char* g_buffer = (char*)malloc(512 * 424 * 20);
	char* b_temp = g_buffer;
	std::cout << "the matrix size: " << MappingMatrix.size() << std::endl;
	for (size_t i = 0; i < 512 * 424; i++)
	{
		//std::cout<<sizeof(float)<<" "<<sizeof(" ")<< " " <<sizeof("\n") <<std::endl;
		float x = MappingMatrix[i].X;
		float y = MappingMatrix[i].Y;
		//if (x >-100000 && x <100000) {
		if (x > 0 && x < 10000) {
			/*std::cout << x << " " << y << std::endl;*/
		}
		else {
			x = -10000;
			y = -10000;
		}
		sprintf(b_temp, "%09.1lf %09.1lf\n", x, y);
		//std::cout << MappingMatrix[i].X << " " << MappingMatrix[i].Y << std::endl;
		b_temp = b_temp + 20;
	}
	FILE* stream;
	stream = fopen((savepath + currtime + "-align.txt").c_str(), "w");
	fwrite(g_buffer, 20 * 512 * 424, 1, stream);
	fclose(stream);
	Sleep(20);
	delete[] g_buffer;
	//g_buffer = NULL;
	//free(g_buffer);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr IGrabber::convertDepthToPointXYZ()
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	int cloudWidth = std::abs(start_x - end_x);
	int cloudHeight = std::abs(start_y - end_y);
	cloud->width = static_cast<uint32_t>(cloudWidth);
	cloud->height = static_cast<uint32_t>(cloudHeight);
	cloud->is_dense = false;*/

	//std::vector<pcl::PointXYZ*> pclList;
	////pcl::PointXYZ* pt = &cloud->points[0];
	//pcl::PointXYZ* pt = NULL;
	typedef pcl::PointXYZRGBA PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	PointCloud::Ptr cloud(new PointCloud);
	////ofstream out("finalpath1.txt");
	////cout << MappingMatrix.size() << endl;
	////cloud->points.resize(MappingMatrix.size());
	////string currenttime = getTime();
	///*ofstream out(savepath + currenttime + "point.txt");*/
	//for (int i = 0; i < MappingMatrix.size(); i++) {
	//	//UINT16 depth = depthImg.data[i];
	//	UINT16 depth;
	//	depth = depthArray[i];
	//	ColorSpacePoint sp = MappingMatrix[i];
	//	//if (sp.X >=start_x && sp.X <= end_x && sp.Y >= start_y && sp.Y <= end_y) {
	//	pcl::PointXYZ point;
	//	DepthSpacePoint depthSpacePoint = { static_cast<float>(i % depth_width), static_cast<float>(i / depth_width) };

	//	// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
	//	CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
	//	m_pMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
	//	point.x = cameraSpacePoint.X;
	//	point.y = cameraSpacePoint.Y;
	//	point.z = cameraSpacePoint.Z;
	//	if (point.z > 0.03)
	//	{
	//		/*out << point.x << " " << point.y << " " << point.z ;
	//		out << "\n";*/

	//		cloud->push_back(point);
	//	}
	//	/*if (point.z > 0.03)
	//	{
	//		cloud->push_back(point);
	//	}*/
	//	//cout << i % depth_width << "  " << i / depth_width << "  " << depth << endl;
	////}
	//}
	////out.close();
	//cout << "the size of the cloud is: " << cloud->size() << std::endl;
	for (int m = 0; m < depthImg.rows; m++)
		for (int n = 0; n < depthImg.cols; n++)
		{
			ushort d = depthImg.ptr<ushort>(m)[n];
			if (d == 0)
				continue;
			pcl::PointXYZRGBA p;
			p.x = m;
			p.y = n;
			p.z = double(d);

			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色。
			p.b = colorImg.ptr<uchar>(m)[n * 3];;
			p.g = colorImg.ptr<uchar>(m)[n * 3 + 1];
			p.r = colorImg.ptr<uchar>(m)[n * 3 + 2];

			//如果不需要将点云上色，可改成单一色。
			//p.b = 255;;
			//p.g = 255;
			//p.r = 255;

			// 把p加入到点云中
			cloud->points.push_back(p);
		}
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	////////////////////////////////////高斯滤波//////////////////////////////////////

	//bool isGaussfilter = false;
	//if (isGaussfilter) {
	//	//Set up the Gaussian Kernel
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud(new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
	//	//Set up the Convolution Filter
	//	pcl::filters::Convolution3D <pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
	//	(*kernel).setSigma(8);

	//	(*kernel).setThresholdRelativeToSigma(5);

	//	std::cout << "Kernel made" << std::endl;

	//	//Set up the KDTreepcl::search::KdTree::Ptr kdtree(new pcl::search::KdTree);
	//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	//	(*kdtree).setInputCloud(cloud);

	//	std::cout << "KdTree made" << std::endl;

	//	//Set up the Convolution Filterpcl::filters::Convolution3D> convolution;
	//	convolution.setKernel(*kernel);
	//	convolution.setInputCloud(cloud);
	//	convolution.setSearchMethod(kdtree);
	//	convolution.setRadiusSearch(5);
	//	std::cout << "Convolution Start" << std::endl;
	//	convolution.convolve(*outputcloud);
	//	std::cout << "Convoluted" << std::endl;
	//	cloud = outputcloud;
	//}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr IGrabber::convertDepthRegionToPointXYZ(const UINT16* depthBuffer, cv::Mat* colorimg)
{
	return pcl::PointCloud<pcl::PointXYZ>::Ptr();
}


pcl::PointCloud<pcl::PointXYZ>::Ptr IGrabber::convertDepthToPointXYZGuanFang(UINT16* depthBuffer)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	cloud->width = static_cast<uint32_t>(depth_width);
	cloud->height = static_cast<uint32_t>(depth_height);
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);

	pcl::PointXYZ* pt = &cloud->points[0];
	for (int y = 0; y < depth_height; y++) {
		for (int x = 0; x < depth_width; x++, pt++) {
			pcl::PointXYZ point;

			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 depth = depthBuffer[y * depth_width + x];

			// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			m_pMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
			point.x = cameraSpacePoint.X;
			point.y = cameraSpacePoint.Y;
			point.z = cameraSpacePoint.Z;

			*pt = point;
		}
	}

	return cloud;
}

////将深度数据转换为cv::mat
//cv::Mat IGrabber::ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
//{
//	cv::Mat img(nHeight, nWidth, CV_8UC3);
//	uchar* p_mat = img.data;
//
//	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
//
//	while (pBuffer < pBufferEnd)
//	{
//		USHORT depth = *pBuffer;
//
//		//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth / 25) : 0);
//		BYTE intensity;
//		if (depth <= nMinDepth) {
//			intensity = 0;
//		}
//		else if (depth >= nMaxDepth) {
//			intensity = 255;
//		}
//		else {
//			intensity = static_cast<BYTE>(255 * (depth - nMinDepth) / (nMaxDepth - nMinDepth));
//		}
//		*p_mat = intensity;
//		p_mat++;
//		*p_mat = intensity;
//		p_mat++;
//		*p_mat = intensity;
//		p_mat++;
//
//		++pBuffer;
//	}
//	return img;
//}
// 转换depth图像到cv::Mat
cv::Mat IGrabber::ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;

		BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;

		++pBuffer;
	}
	return img;
}

cv::Mat IGrabber::ConvertDepthMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_16UC1);
	UINT16* p_mat = (UINT16*)img.data;
	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		UINT16 depth = *pBuffer;

		//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth / 25) : 0);
		UINT16 intensity = static_cast<USHORT> (depth);


		*p_mat = intensity;
		p_mat++;
		++pBuffer;
	}
	return img;
}
////将彩色图像转换为cv::Mat
//cv::Mat IGrabber::ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
//{
//	cv::Mat img(nHeight, nWidth, CV_8UC3);
//	uchar* p_mat = img.data;
//
//	const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);
//
//	while (pBuffer < pBufferEnd)
//	{
//		*p_mat = pBuffer->rgbBlue;
//		p_mat++;
//
//		*p_mat = pBuffer->rgbGreen;
//		p_mat++;
//		*p_mat = pBuffer->rgbRed;
//		p_mat++;
//		++pBuffer;
//	}
//	return img;
//
//}
// 转换color图像到cv::Mat
cv::Mat IGrabber::ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
		p_mat++;

		++pBuffer;
	}
	return img;
}


UINT16* IGrabber::GetDepthbuffer()
{
	return pBuffer_depth;
}

bool IGrabber::isMappingMatrixempty()
{
	int a = MappingMatrix.size();
	if (a == 0) return true;
	else return false;
}

string IGrabber::getTime()
{
	time_t tt = time(NULL);
	struct tm* stm = localtime(&tt);
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	char tmp[32];
	sprintf(tmp, "%04d-%02d-%02d-%02d-%02d-%02d-%03d", 1900 + stm->tm_year, 1 + stm->tm_mon, stm->tm_mday, stm->tm_hour,
		stm->tm_min, stm->tm_sec, sys.wMilliseconds);
	printf(tmp);
	return tmp;
}