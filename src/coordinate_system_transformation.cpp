#include<iostream>
#include<cmath>
#include "quaternion.h"
 
using namespace std;
 
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
//  - frame_current_euler : 当前坐标系相对于某参考系的 ZYX 欧拉角 (yaw,pitch,roll)
//  - frame_target_euler  : 目标坐标系相对于同一参考系的 ZYX 欧拉角
//  - t_diff : 目标坐标系原点在参考系下的位置减去当前坐标系原点在参考系下的位置（位置差），在参考系下表示
// 输出:
//  - out_pos : 在目标坐标系下的位姿位置
//  - out_euler: 在目标坐标系下的姿态 (yaw,pitch,roll)
static void transformPose(const cv::Vec3d &pos, const cv::Vec3d &euler,
						  const cv::Vec3d &frame_current_euler, const cv::Vec3d &frame_target_euler,
						  const cv::Vec3d &t_diff,
						  cv::Vec3d &out_pos, cv::Vec3d &out_euler) {
	// 计算当前/目标坐标系的旋转四元数（相对于相同参考系）
	Quaternion q_currFrame = quaternionFromEulerZYX(frame_current_euler[0], frame_current_euler[1], frame_current_euler[2]);
	Quaternion q_tgtFrame = quaternionFromEulerZYX(frame_target_euler[0], frame_target_euler[1], frame_target_euler[2]);

	// 相对旋转：将当前坐标系的向量转换到目标坐标系的旋转
	Quaternion q_rel = q_tgtFrame * q_currFrame.inverse();
	q_rel.normalize();

	// 旋转位置向量并加上位置差（位置差应为目标原点在参考系下减去当前原点在参考系下）
	cv::Vec3d rotated_pos = rotateVector(pos, q_rel);
	out_pos = rotated_pos + t_diff;

	// 姿态变换：新姿态四元数 = q_rel * q_pose
	Quaternion q_pose = quaternionFromEulerZYX(euler[0], euler[1], euler[2]);
	Quaternion q_new = q_rel * q_pose;
	q_new.normalize();
	out_euler = quaternionToEulerZYX(q_new);
}

// 简单示例（可被替换为更完整的接口）
int main_example_transform() {
	cv::Vec3d pos(1.0, 0.0, 0.0);
	cv::Vec3d euler_pose(0.0, 0.0, 0.0); // yaw,pitch,roll

	// 假设当前坐标系与参考系对齐，目标坐标系绕 z 轴 90deg，并往 x 方向平移 1
	cv::Vec3d frame_curr(0.0, 0.0, 0.0);
	cv::Vec3d frame_tgt(M_PI/2.0, 0.0, 0.0);
	cv::Vec3d tdiff(1.0, 0.0, 0.0);

	cv::Vec3d out_pos, out_euler;
	transformPose(pos, euler_pose, frame_curr, frame_tgt, tdiff, out_pos, out_euler);
	cout << "Out pos: " << out_pos << "\n";
	cout << "Out euler (yaw,pitch,roll): " << out_euler << "\n";
	return 0;
}


