#pragma warning(disable:4996)
//------------------------------------------------------------------------------
// read data from kinect, convert the depth data and color data to cv::mat and save it
// conver the depth data of the specified area to the point cloud  in the camera coordinate
// editor: SUN CHANG JIANG
// date : 2020-12-5
//------------------------------------------------------------------------------
// 

#include "IGrabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h>//PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include "imageProcess.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
int main() {
	bool isInitSuccess = false;
	//�����Ƿ񱣴����ʾ��ȺͲ�ɫͼƬ
	bool IsSaveImg = true;
	bool IsShowImg = true;
	IGrabber myGrabber;
	//pcl viewer��ʼ����������������
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGBA> fildColor(mycloud, "z"); // ����z�ֶν�����Ⱦ
	//viewer->addPointCloud<pcl::PointXYZRGBA>(mycloud, fildColor, "cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud"); // ���õ��ƴ�С

	PointCloud::Ptr mycloud(new PointCloud);
	imageProcess imageProcessor;


	//��������ʼ��
	while (!isInitSuccess) {
		isInitSuccess = myGrabber.SensorInit();
		std::cout << "trying to init" << std::endl;
	}
	std::cout << "init success" << std::endl;
	Sleep(500);
	//loop

	string basepath = ".\\kinect-data\\";
	string begintime = myGrabber.getTime();
	//��ʼʱ��
	string savepath = basepath + begintime;
	string command;
	if (IsSaveImg) {
		command = "mkdir -p " + savepath;
		system(command.c_str());
	}
	int i = 0;
	while (true) {
		clock_t start_time = clock(); //���ڼ����������ʱ��
		 myGrabber.GetImg(IsSaveImg,IsShowImg);
		if (myGrabber.isMappingMatrixempty()) 
			continue;
		if (IsSaveImg) {
			myGrabber.SaveImg(command,savepath);
		}
		Sleep(100);
		//imshow("Thresholded Image", imgThresholded); //show the thresholded image
		//imshow("Original", myGrabber.colorImg); //show the original image
		//pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
		//cout << i << endl;
		//imageProcessor.processImageGetGreenRegion(myGrabber.colorImg);
		////cout << imageProcessor.rectPoint[0][0].x<< " "<<imageProcessor.rectPoint[0][0].y<< endl;
		//cout << imageProcessor.rectPoint.size() << endl;
		//for (auto& rect : imageProcessor.rectPoint) {
		//	mycloud = myGrabber.convertDepthToPointXYZ(rect.at(0).x, rect.at(0).y, rect.at(1).x, rect.at(1).y,savepath);
		//}
		mycloud = myGrabber.convertDepthToPointXYZ();
	    myGrabber.deletedepth();
		//�ļ�����
		string currtime = to_string(i);
		//����洢·��
		//string savepath = basepath + begintime + "\\";
		if (IsSaveImg)
			pcl::io::savePCDFileASCII(savepath + "\\" + currtime + "test_pcb", *mycloud);

		//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//ֱ�Ӵ���һ����ʾ����

		//viewer.showCloud(mycloud);

		mycloud->points.clear();
		clock_t end_time = clock();
		cout << "The run time is: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl; //�����������ʱ��(2021/9/9)
		i++;
	}
	return 0;
}
