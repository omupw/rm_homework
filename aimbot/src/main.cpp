#include <iostream>
#include <opencv2/opencv.hpp>
#include "detect.hpp"

int main()
{
	// Load the video
    cv::VideoCapture cap("../data/armor/avi.mp4");
	if(!cap.isOpened())
	{
		std::cerr << "无法打开视频 " << std::endl;
		return -1;
	}

	DetectParams params; // 可在这里调整参数
	cv::Mat frame, debug;// debug 用于绘制结果显示

	// solvepnp 需要的相机内参和畸变参数，装甲板物理尺寸（单位：米）
	cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
		1567.7907457705167, 0, 662.3933648922284,
		0.0, 1564.9113082257936, 536.8662848443158,
		0.0, 0.0, 1.0);
	cv::Mat distCoeffs = (cv::Mat_<double>(5,1) << 
		-0.0682737005569565, 0.1983544402464456, 0.0016855914452479342, 0.0024125119646311016, 0.0);
	const float SMALL_ARMOR_WIDTH  = 0.135f; 
	const float SMALL_ARMOR_HEIGHT = 0.055f; 

	while(true)
	{
		if(!cap.read(frame) || frame.empty()) break;//无法读取或读完退出

		std::vector<Armor> armors = detectFrame(frame, debug, params);
		//处理图像，筛选灯条，配对装甲板返回，并在 debug 上绘制结果
		// 对每个检测到的装甲求解位姿 rvec, tvec
		for(size_t i=0;i<armors.size();++i)
		{
			const Armor& a = armors[i];
			cv::Mat rvec, tvec;
			bool ok = solveArmorPose(a, cameraMatrix, distCoeffs, SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT, rvec, tvec);
            if(!ok)
            {
                std::cout<<"装甲板 "<<i<<" 位姿求解失败"<<std::endl;
                continue;
            }
            cv::Affine3d T(rvec, tvec);
            cv::Point3d pt_cam = T * cv::Point3d(0, 0, 0);
			
		}

		cv::imshow("detect", debug);// 显示检测结果

		//按 'q' 键退出
		if (cv::waitKey(30) == 'q') {
			break;
		}
	}

	cap.release();
	cv::destroyAllWindows();
	return 0;
}
