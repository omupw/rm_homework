#include <iostream>
#include <opencv2/opencv.hpp>
#include "quaternion.h"

int main() {
    // 方向向量
    cv::Mat v = (cv::Mat_<double>(3,1) << 1.0, 0.0, 0.0);
    Quaternion qv(v); // 纯四元数

    // 绕 z 轴 90 度 的旋转四元数
    cv::Mat axis = (cv::Mat_<double>(3,1) << 0.0, 0.0, 1.0);
    double ang = CV_PI / 2.0;
    Quaternion qrot = Quaternion::fromAxisAngle(axis, ang);

    Quaternion qres = Quaternion::rotateQuaternion(qv, qrot);
    cv::Mat out = qres.toCvMat();
    std::cout << "Rotated vector: [" << out.at<double>(0) << ", " << out.at<double>(1) << ", " << out.at<double>(2) << "]\n";
    return 0;
}
