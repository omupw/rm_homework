#include<iostream>
#include<cmath>
#include "quaternion.h"
#include <opencv2/opencv.hpp>
 
using namespace std;
 
//编译命令: g++ -o bin/coordinate_system_transformation src/coordinate_system_transformation.cpp src/quaternion.cpp  -I ./include `pkg-config --cflags --libs opencv4`

// 将 ZYX (yaw, pitch, roll) 欧拉角转换为四元数
static Quaternion quaternionFromEulerZYX(double yaw, double pitch, double roll) {
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	double w = cr * cp * cy + sr * sp * sy;
	double x = sr * cp * cy - cr * sp * sy;
	double y = cr * sp * cy + sr * cp * sy;
	double z = cr * cp * sy - sr * sp * cy;
	return Quaternion(w, x, y, z);
}

// 将四元数转换为 ZYX (yaw, pitch, roll)
static cv::Vec3d quaternionToEulerZYX(const Quaternion &q) {
	double w = q.w, x = q.x, y = q.y, z = q.z;
	// roll (x-axis rotation)
	double sinr_cosp = 2.0 * (w * x + y * z);
	double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
	double roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2.0 * (w * y - z * x);
	double pitch;
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2.0, sinp);
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2.0 * (w * z + x * y);
	double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
	double yaw = atan2(siny_cosp, cosy_cosp);

	return cv::Vec3d(yaw, pitch, roll);
}

// 旋转向量 v (3x1) 使用旋转四元数 rot (单位四元数)，返回旋转后的向量
static cv::Vec3d rotateVector(const cv::Vec3d &v, const Quaternion &rot) {
	cv::Mat vm = (cv::Mat_<double>(3,1) << v[0], v[1], v[2]);
	Quaternion pv(vm); // 纯四元数
	Quaternion r = Quaternion::rotateQuaternion(pv, rot);
	cv::Mat out = r.toCvMat();
	return cv::Vec3d(out.at<double>(0), out.at<double>(1), out.at<double>(2));
}

// 封装的坐标系转换函数
// 输入:
//  - pos : 当前位姿位置 (x,y,z)
//  - euler : 当前位姿姿态 (yaw, pitch, roll) （ZYX）
//  - t_diff : 目标坐标系原点在参考系下的位置减去当前坐标系原点在参考系下的位置（位置差），在参考系下表示
// 输出:
//  - out_pos : 在目标坐标系下的位姿位置
//  - out_euler: 在目标坐标系下的姿态 (yaw,pitch,roll)
// 封装的坐标系转换函数（使用已计算好的相对旋转四元数 `q_rel`）
// 输入:
//  - pos : 当前位姿位置 (x,y,z)
//  - euler : 当前位姿姿态 (yaw, pitch, roll) （ZYX）
//  - q_rel : 当前坐标系到目标坐标系的相对旋转（单位四元数）
//  - t_diff : 目标坐标系原点在参考系下的位置减去当前坐标系原点在参考系下的位置（位置差），在参考系下表示
// 输出:
//  - out_pos : 在目标坐标系下的位姿位置
//  - out_euler: 在目标坐标系下的姿态 (yaw,pitch,roll)
static void transformPose(const cv::Vec3d &pos, const cv::Vec3d &euler,
						  const Quaternion &q_rel, const cv::Vec3d &t_diff,
						  cv::Vec3d &out_pos, cv::Vec3d &out_euler) {
	// 假定 q_rel 已为单位四元数并表示从当前坐标系到目标坐标系的旋转
	Quaternion q_rel_norm = q_rel;
	q_rel_norm.normalize();

	// 旋转位置向量并加上位置差（位置差应为目标原点在参考系下减去当前原点在参考系下）
	cv::Vec3d rotated_pos = rotateVector(pos, q_rel_norm);
	out_pos = rotated_pos + t_diff;

	// 姿态变换：新姿态四元数 = q_rel * q_pose
	Quaternion q_pose = quaternionFromEulerZYX(euler[0], euler[1], euler[2]);
	Quaternion q_new = q_rel_norm * q_pose;
	q_new.normalize();
	out_euler = quaternionToEulerZYX(q_new);
}



int main() {
	// 第一行：6 个量 -> pos.x pos.y pos.z yaw pitch roll
	// 第二行：6 个量 -> t_diff.x t_diff.y t_diff.z  rel_yaw rel_pitch rel_roll

	double px, py, pz, yaw, pitch, roll;
	if (!(cin >> px >> py >> pz >> yaw >> pitch >> roll)) {
		cerr << "Expect 6 numbers on first line: px py pz yaw pitch roll\n";
		return 1;
	}
	cv::Vec3d pos(px, py, pz);
	cv::Vec3d euler_pose(yaw, pitch, roll);

	double td0, td1, td2, ryaw, rpitch, rroll;
	if (!(cin >> td0 >> td1 >> td2 >> ryaw >> rpitch >> rroll)) {
		cerr << "Expect 6 numbers on second line: tdiff_x tdiff_y tdiff_z rel_yaw rel_pitch rel_roll\n";
		return 1;
	}
	cv::Vec3d t_diff(td0, td1, td2);

	Quaternion q_rel = quaternionFromEulerZYX(ryaw, rpitch, rroll);
	q_rel.normalize();

	cv::Vec3d out_pos, out_euler;
	transformPose(pos, euler_pose, q_rel, t_diff, out_pos, out_euler);
	//输出精度设置为小数点后2位，使用固定小数点格式
	cout.setf(std::ios::fixed);
	cout.precision(2);
	cout << out_pos[0] << " " << out_pos[1] << " " << out_pos[2] << " ";
	cout << out_euler[0] << " " << out_euler[1] << " " << out_euler[2] << "\n";

	return 0;
}




