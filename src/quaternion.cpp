#include "quaternion.h"
#include <cmath>

Quaternion::Quaternion()
    : w(1.0), x(0.0), y(0.0), z(0.0) {}

Quaternion::Quaternion(double w_, double x_, double y_, double z_)
    : w(w_), x(x_), y(y_), z(z_) {}

Quaternion::Quaternion(const cv::Mat &vec_xyz) {
    CV_Assert(!vec_xyz.empty());
    cv::Mat v = vec_xyz.reshape(1, 1); // flatten
    CV_Assert(v.cols >= 3);
    x = v.at<double>(0);
    y = v.at<double>(1);
    z = v.at<double>(2);
    w = 0.0;
}

Quaternion Quaternion::fromAxisAngle(const cv::Mat &axis, double angle_rad) {
    CV_Assert(!axis.empty());
    cv::Mat a = axis.reshape(1, 1);
    CV_Assert(a.cols >= 3);
    double ax = a.at<double>(0);
    double ay = a.at<double>(1);
    double az = a.at<double>(2);
    double len = std::sqrt(ax*ax + ay*ay + az*az);
    if (len == 0.0) {
        return Quaternion();
    }
    ax /= len; ay /= len; az /= len;
    double half = angle_rad * 0.5;
    double s = std::sin(half);
    double qw = std::cos(half);
    return Quaternion(qw, ax * s, ay * s, az * s);
}
//加
Quaternion Quaternion::operator+(const Quaternion &o) const {
    return Quaternion(w + o.w, x + o.x, y + o.y, z + o.z);
}
//减
Quaternion Quaternion::operator-(const Quaternion &o) const {
    return Quaternion(w - o.w, x - o.x, y - o.y, z - o.z);
}
//乘
Quaternion Quaternion::operator*(const Quaternion &o) const {
    // Hamilton product
    return Quaternion(
        w*o.w - x*o.x - y*o.y - z*o.z,
        w*o.x + x*o.w + y*o.z - z*o.y,
        w*o.y - x*o.z + y*o.w + z*o.x,
        w*o.z + x*o.y - y*o.x + z*o.w
    );
}
//共轭
Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}
//范数（长度）
double Quaternion::norm() const {
    return std::sqrt(w*w + x*x + y*y + z*z);
}
//逆
Quaternion Quaternion::inverse() const {
    double n2 = w*w + x*x + y*y + z*z;
    if (n2 == 0.0) return Quaternion();
    Quaternion c = conjugate();
    return Quaternion(c.w / n2, c.x / n2, c.y / n2, c.z / n2);
}
//归一化
void Quaternion::normalize() {
    double n = norm();
    if (n == 0.0) return;
    w /= n; x /= n; y /= n; z /= n;
}
//旋转四元数对表示方向的纯四元数进行旋转
Quaternion Quaternion::rotateQuaternion(const Quaternion &dirQuat, const Quaternion &rotQuat) {
    Quaternion r = rotQuat;
    r.normalize();
    return r * dirQuat * r.inverse();
}
//转换为 cv::Mat (3x1) 的向量，仅返回 xyz 部分
cv::Mat Quaternion::toCvMat() const {
    cv::Mat m(3, 1, CV_64F);
    m.at<double>(0) = x;
    m.at<double>(1) = y;
    m.at<double>(2) = z;
    return m;
}
