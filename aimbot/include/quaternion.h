#pragma once

#include <opencv2/opencv.hpp>

class Quaternion {
public:
    double w, x, y, z;

    // 默认构造为单位四元数
    Quaternion();
    Quaternion(double w_, double x_, double y_, double z_);

    // 从含有 xyz 的 cv::Mat 构造（单列或单行向量），转为纯四元数 w=0
    explicit Quaternion(const cv::Mat &vec_xyz);

    // 从方向向量（cv::Mat）和角度（rad）构造旋转四元数
    static Quaternion fromAxisAngle(const cv::Mat &axis, double angle_rad);

    // 四元数运算
    Quaternion operator+(const Quaternion &o) const;
    Quaternion operator-(const Quaternion &o) const;
    Quaternion operator*(const Quaternion &o) const;

    // 共轭、范数、逆、归一化
    Quaternion conjugate() const;
    double norm() const;
    Quaternion inverse() const;
    void normalize();

    // 使用旋转四元数对表示方向的纯四元数进行旋转： q_rot * q_dir * q_rot^{-1}
    static Quaternion rotateQuaternion(const Quaternion &dirQuat, const Quaternion &rotQuat);

    // 从 ZYX (yaw, pitch, roll) 欧拉角构造四元数（rad）
    static Quaternion fromEulerZYX(double yaw, double pitch, double roll);

    // 将四元数转换为 ZYX (yaw, pitch, roll) 欧拉角，返回 (yaw, pitch, roll)
    static cv::Vec3d toEulerZYX(const Quaternion &q);

    // 使用旋转四元数对向量进行旋转（输入与输出为 cv::Vec3d）
    static cv::Vec3d rotateVector(const cv::Vec3d &v, const Quaternion &rot);

    // 转换为 cv::Mat (3x1) 的向量，仅返回 xyz 部分
    cv::Mat toCvMat() const;
};
