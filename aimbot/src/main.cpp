#include <iostream>
#include <opencv2/opencv.hpp>
#include "detect.hpp"
#include "quaternion.h"
#include <cmath>
#include <vector>

// 简单 Kalman Filter（状态 6: px,py,pz,vx,vy,vz），观测 3: px,py,pz
struct SimpleKalman {
	cv::Mat x; // 6x1 state
	cv::Mat P; // 6x6 cov
	cv::Mat F; // 6x6
	cv::Mat Q; // 6x6
	cv::Mat H; // 3x6
	cv::Mat R; // 3x3

	SimpleKalman() {
		x = cv::Mat::zeros(6,1,CV_64F);
		P = cv::Mat::eye(6,6,CV_64F);
		F = cv::Mat::eye(6,6,CV_64F);
		// F with dt=1 assumed
		F.at<double>(0,3) = 1.0;
		F.at<double>(1,4) = 1.0;
		F.at<double>(2,5) = 1.0;
		H = cv::Mat::zeros(3,6,CV_64F);
		H.at<double>(0,0) = 1.0;
		H.at<double>(1,1) = 1.0;
		H.at<double>(2,2) = 1.0;
		Q = cv::Mat::eye(6,6,CV_64F);
		R = cv::Mat::eye(3,3,CV_64F);
	}

	void initWithMeasurement(const cv::Vec3d &m) {
		x.at<double>(0) = m[0];
		x.at<double>(1) = m[1];
		x.at<double>(2) = m[2];
		x.at<double>(3) = 0.0;
		x.at<double>(4) = 0.0;
		x.at<double>(5) = 0.0;
		P = cv::Mat::eye(6,6,CV_64F);
	}

	cv::Vec3d predictUpdate(const cv::Vec3d &measurement, double process_variance, double measurement_variance) {
		// set Q and R based on variances (use same structure as python file)
		cv::Mat Qtmp = (cv::Mat_<double>(6,6) <<
			1.0/20.0, 0, 0, 1.0/8.0, 0, 0,
			0, 1.0/20.0, 0, 0, 1.0/8.0, 0,
			0, 0, 1.0/20.0, 0, 0, 1.0/8.0,
			1.0/8.0, 0, 0, 1.0/3.0, 0, 0,
			0, 1.0/8.0, 0, 0, 1.0/3.0, 0,
			0, 0, 1.0/8.0, 0, 0, 1.0/3.0);
		Q = Qtmp * process_variance;
		R = cv::Mat::eye(3,3,CV_64F) * measurement_variance;

		// Predict
		x = F * x;
		P = F * P * F.t() + Q;

		// Update
		cv::Mat S = H * P * H.t() + R; // 3x3
		cv::Mat K = P * H.t() * S.inv(); // 6x3
		cv::Mat z = (cv::Mat_<double>(3,1) << measurement[0], measurement[1], measurement[2]);
		x = x + K * (z - H * x);
		P = (cv::Mat::eye(6,6,CV_64F) - K * H) * P;

		return cv::Vec3d(x.at<double>(0), x.at<double>(1), x.at<double>(2));
	}
};

int main()
{
	// 读取从相机坐标系到目标坐标系的相对位移和平移（在程序开始时手动输入）
	// 输入格式（空格分隔，回车结束）：td_x td_y td_z rel_yaw rel_pitch rel_roll
	// 角度单位：弧度
	std::cout << "请输入目标坐标系相对于相机坐标系的 t_diff(x y z) 和 rel yaw pitch roll (弧度):\n";
	double td_x=0, td_y=0, td_z=0;
	double rel_yaw=0, rel_pitch=0, rel_roll=0;
	if(!(std::cin >> td_x >> td_y >> td_z >> rel_yaw >> rel_pitch >> rel_roll)){
		std::cerr << "输入无效，使用默认零变换\n";
		td_x = td_y = td_z = rel_yaw = rel_pitch = rel_roll = 0.0;
	}
	cv::Vec3d t_diff(td_x, td_y, td_z);

	Quaternion q_rel = Quaternion::fromEulerZYX(rel_yaw, rel_pitch, rel_roll);
	q_rel.normalize();
    
	// 卡尔曼滤波初始化（状态：位置(x,y,z) 和 速度(vx,vy,vz)）
	double process_variance = 1e-3; 
	double measurement_variance = 1e-2;
	SimpleKalman kf;
	bool kf_inited = false;
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
	const float SMALL_ARMOR_WIDTH  = 0.135f; // 与 python 中一致
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
			// 相机系下点
			cv::Vec3d pos_cam(pt_cam.x, pt_cam.y, pt_cam.z);
			// 使用四元数旋转并加平移，得到目标坐标系下的坐标
			cv::Vec3d out_pos = Quaternion::rotateVector(pos_cam, q_rel) + t_diff;
			// 根据目标坐标系下的位置计算 yaw 和 pitch（假设相机/目标前向为 +Z，x 右，y 下）
			double yaw_deg = atan2(out_pos[0], out_pos[2]) * 180.0 / M_PI; // 横向角度
			double pitch_deg = -atan2(out_pos[1], out_pos[2]) * 180.0 / M_PI; // 俯仰角
			// 使用卡尔曼滤波器处理目标坐标（作为观测值）
			cv::Vec3d filt_pos;
			if(!kf_inited) {
				kf.initWithMeasurement(out_pos);
				kf_inited = true;
				filt_pos = out_pos;
			} else {
				filt_pos = kf.predictUpdate(out_pos, process_variance, measurement_variance);
			}
			double filt_yaw_deg = atan2(filt_pos[0], filt_pos[2]) * 180.0 / M_PI;
			double filt_pitch_deg = atan2(filt_pos[1], filt_pos[2]) * 180.0 / M_PI;

			// 实时输出：装甲索引、相机系坐标 -> 目标系坐标(raw) -> 目标系坐标(filtered) -> yaw/pitch
			std::cout.setf(std::ios::fixed);
			std::cout.precision(3);
			std::cout << "Armor " << i << ": cam_xyz=" << pos_cam[0] << "," << pos_cam[1] << "," << pos_cam[2]
				<< "  raw_tgt_xyz=" << out_pos[0] << "," << out_pos[1] << "," << out_pos[2]
				<< "  filt_tgt_xyz=" << filt_pos[0] << "," << filt_pos[1] << "," << filt_pos[2]
				<< "  raw(y,p)=" << yaw_deg << "," << pitch_deg
				<< "  filt(y,p)=" << filt_yaw_deg << "," << filt_pitch_deg << "\n";
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
