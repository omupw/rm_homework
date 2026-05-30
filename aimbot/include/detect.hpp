#ifndef AIMBOT_DETECT_HPP
#define AIMBOT_DETECT_HPP

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

// 灯条结构体
struct LightBar {
    cv::RotatedRect rect; // 最小外接旋转矩形
    float length;         // 长边
    float width;          // 短边
    float angle;          // 与水平的角度（度数）
    cv::Point2f center;   // 中心
    cv::Point2f top;      // 上端点
    cv::Point2f bottom;   // 下端点
    float area;           // 轮廓面积
};

// 装甲板结构体，按要求：左上，左下，右上，右下 顺序
struct Armor {
    LightBar left_light;
    LightBar right_light;
    std::vector<cv::Point2f> image_points; // 4 点
    cv::Point2f center;
};

// 参数结构体，便于调整
struct DetectParams {
    // 红色两段 HSV 阈值（默认值可调）
    cv::Scalar red_lower1 = cv::Scalar(0, 70, 70);
    cv::Scalar red_upper1 = cv::Scalar(10, 255, 255);
    cv::Scalar red_lower2 = cv::Scalar(170, 70, 70);
    cv::Scalar red_upper2 = cv::Scalar(180, 255, 255);

    // 形态学核大小
    int open_kernel = 3;
    int close_kernel = 5;

    // 面积筛选
    float min_area = 30.0f;
    float max_area = 5000.0f;

    // 灯条长宽比
    float min_light_ratio = 1.5f; // length / width
    float max_light_ratio = 15.0f;

    // 灯条最大倾角（与竖直方向偏差）
    float max_light_angle = 30.0f; // degrees

    // 配对筛选
    float max_center_y_diff = 40.0f; // 垂直中心差
    float max_angle_diff = 20.0f;    // 灯条角度差
    float min_height_ratio = 0.3f;   // 高度比（short/long）
    float min_armor_ratio = 0.5f;    // 装甲宽高比 min
    float max_armor_ratio = 8.0f;    // 装甲宽高比 max

    // 灯条间距倍数范围（相对于平均高度）
    float min_gap_mul = 0.5f;
    float max_gap_mul = 8.0f;

};

// 主检测接口：对一帧图像进行检测，返回装甲列表并在 debug_image 上绘制结果
std::vector<Armor> detectFrame(const cv::Mat& frame, cv::Mat& debug_image, const DetectParams& params);

// 根据装甲物理尺寸与四个图像角点，以及相机内参和畸变，求解位姿（rvec, tvec）
// 假设装甲板坐标系：中心为原点，xy 平面与装甲板平行，角点顺序为：左上，左下，右上，右下
bool solveArmorPose(const Armor& armor, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                    float armor_width, float armor_height,
                    cv::Mat& rvec, cv::Mat& tvec);

#endif // AIMBOT_DETECT_HPP
