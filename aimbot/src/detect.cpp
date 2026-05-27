#include "detect.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

//图像处理
static void preprocess(const cv::Mat& frame, cv::Mat& closed, const DetectParams& params)
{
    cv::Mat blurred, hsv, binary1, binary2;//高斯模糊，HSV，二值图1，二值图2（注：红色有两段）
    cv::GaussianBlur(frame, blurred, cv::Size(3,3), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, params.red_lower1, params.red_upper1, binary1);
    cv::inRange(hsv, params.red_lower2, params.red_upper2, binary2);
    cv::bitwise_or(binary1, binary2, binary1);//合并两段红色

    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.open_kernel, params.open_kernel));
    cv::morphologyEx(binary1, closed, cv::MORPH_OPEN, open_kernel);//开运算去噪
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.close_kernel, params.close_kernel));
    cv::morphologyEx(closed, closed, cv::MORPH_CLOSE, close_kernel);//闭运算填充
}

static bool isValidLightBar(const LightBar& light, const DetectParams& params)
{
    if(light.area < params.min_area || light.area > params.max_area) return false;
    float ratio = light.length / light.width;
    if(ratio < params.min_light_ratio || ratio > params.max_light_ratio) return false;
    // 期望灯条接近竖直，计算与竖直方向的差值
    float abs_angle = std::abs(light.angle);
    float angle_from_vertical = std::abs(90.0f - abs_angle);//要从横向变成竖向的角度差
    if(angle_from_vertical > params.max_light_angle) return false;
    return true;
}

static void getLightEndpoints(LightBar& light)
{
    cv::Point2f verts[4];
    light.rect.points(verts);
    // 按 y 排序,在图像坐标系下，y 越小越靠上
    std::sort(verts, verts+4, [](const cv::Point2f&a, const cv::Point2f&b){ return a.y < b.y; });
    light.top = (verts[0] + verts[1]) * 0.5f;
    light.bottom = (verts[2] + verts[3]) * 0.5f;
    if(light.top.y > light.bottom.y) std::swap(light.top, light.bottom);
}

static std::vector<LightBar> findLightBars(const cv::Mat& closed, const DetectParams& params)
{
    std::vector<std::vector<cv::Point>> contours;//轮廓数组
    cv::findContours(closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<LightBar> lights;//灯条数组
    for(const auto& c : contours)
    {
        //通过面积筛选
        float area = std::abs((float)cv::contourArea(c));
        if(area < params.min_area || area > params.max_area) continue;

        cv::RotatedRect rect = cv::minAreaRect(c);//最小外接旋转矩形

        //给灯条赋值
        LightBar light;
        light.rect = rect;
        light.center = rect.center;
        light.area = area;

        //宽小，长大
        light.width = std::min(rect.size.width, rect.size.height);
        light.length = std::max(rect.size.width, rect.size.height);

        // 先计算上下端点，再用上下端点坐标计算长边与 x 轴的角度（范围 0-180）
        //注：opencv的 angle 定义比较特殊，直接使用可能不太方便，所以这里重新计算角度
        getLightEndpoints(light);//获得灯条上下端点
        // 用上下端点向量（bottom - top）计算角度
        float dx = light.bottom.x - light.top.x;
        float dy = light.bottom.y - light.top.y;
        float angle_rad = std::atan2(dy, dx); // -pi..pi
        float angle_deg = angle_rad * 180.0f / static_cast<float>(CV_PI); // -180..180
        if(angle_deg < 0.0f) angle_deg += 180.0f; // 归一到 0..180
        if(angle_deg >= 180.0f) angle_deg -= 180.0f;
        light.angle = angle_deg;
        if(isValidLightBar(light, params))
        {
            lights.push_back(light);
        }
    }
    // 按 x 排序（从左到右）
    std::sort(lights.begin(), lights.end(), [](const LightBar&a, const LightBar&b){ return a.center.x < b.center.x; });
    return lights;
}

static std::vector<Armor> matchArmors(std::vector<LightBar>& lights, const DetectParams& params)
{
    std::vector<Armor> armors;
    size_t n = lights.size();
    std::vector<bool> used(n, false);
    for(size_t i=0;i<n;i++)
    {
        if(used[i]) continue;
        for(size_t j=i+1;j<n;j++)
        {
            if(used[j]) continue;
            const LightBar& L = lights[i];
            const LightBar& R = lights[j];
            // center y 差
            float center_y_diff = std::abs(L.center.y - R.center.y);
            if(center_y_diff > params.max_center_y_diff) continue;
            // 角度差：取 0..90 的最小差值再比较阈值
            float angle_diff = std::abs(L.angle - R.angle);
            if(angle_diff > 90.0f) angle_diff = 180.0f - angle_diff; // 对称处理
            if(angle_diff > params.max_angle_diff) continue;
            // 高度比
            float height_ratio = std::min(L.length, R.length) / std::max(L.length, R.length);
            if(height_ratio < params.min_height_ratio) continue;
            // 装甲宽高比
            float armor_width = std::abs(R.center.x - L.center.x);
            float avg_height = (L.length + R.length) * 0.5f;
            if(avg_height <= 0.0f) continue;
            float armor_ratio = armor_width / avg_height;
            if(armor_ratio < params.min_armor_ratio || armor_ratio > params.max_armor_ratio) continue;
            // 灯条间距
            if(armor_width < avg_height * params.min_gap_mul || armor_width > avg_height * params.max_gap_mul) continue;

            // 通过：构造 Armor
            Armor a;
            a.left_light = L;
            a.right_light = R;
            // 要求顺序为 左上, 左下, 右上, 右下
            a.image_points.resize(4);
            a.image_points[0] = L.top;   // 左上
            a.image_points[1] = L.bottom; // 左下
            a.image_points[2] = R.top;   // 右上 
            a.image_points[3] = R.bottom; // 右下
            a.center = (L.center + R.center) * 0.5f;
            armors.push_back(a);
            // 标记已用，防止灯条重复组成其他装甲
            used[i] = true;
            used[j] = true;
            break; // i 已匹配，跳到下一个 i
        }
    }
    return armors;
}

std::vector<Armor> detectFrame(const cv::Mat& frame, cv::Mat& debug_image, const DetectParams& params)
{
    std::vector<Armor> result;
    if(frame.empty()) return result;
    
    cv::Mat closed;
    preprocess(frame, closed, params);
    std::vector<LightBar> lights = findLightBars(closed, params);
    // 复制用于匹配的列表（匹配时会用到 used 标记）
    std::vector<LightBar> lights_copy = lights;
    std::vector<Armor> armors = matchArmors(lights_copy, params);

    debug_image = frame.clone();
    // 绘制灯条（绿色）
    for(size_t idx=0; idx<lights.size(); ++idx)
    {
        const LightBar& L = lights[idx];
        cv::Point2f verts[4];
        L.rect.points(verts);
        for(int k=0;k<4;k++) cv::line(debug_image, verts[k], verts[(k+1)%4], cv::Scalar(0,255,0), 2);
        cv::circle(debug_image, L.center, 3, cv::Scalar(0,255,0), -1);
        cv::putText(debug_image, std::to_string((int)idx), L.center + cv::Point2f(8,0), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255), 2);
    }

    // 绘制装甲（红色），对角线与中心点
    for(size_t i=0;i<armors.size();++i)
    {
        const Armor& a = armors[i];
        if(a.image_points.size() == 4)
        {
            // 绘制边
            cv::line(debug_image, a.image_points[0], a.image_points[1], cv::Scalar(0,0,255), 2);
            cv::line(debug_image, a.image_points[1], a.image_points[3], cv::Scalar(0,0,255), 2);
            cv::line(debug_image, a.image_points[3], a.image_points[2], cv::Scalar(0,0,255), 2);
            cv::line(debug_image, a.image_points[2], a.image_points[0], cv::Scalar(0,0,255), 2);
            // 对角线
            cv::line(debug_image, a.image_points[0], a.image_points[3], cv::Scalar(0,0,255), 1);
            cv::line(debug_image, a.image_points[1], a.image_points[2], cv::Scalar(0,0,255), 1);
            // 中心
            cv::circle(debug_image, a.center, 4, cv::Scalar(0,255,255), -1);
        }
    }
    return armors;
}

// 求装甲板位姿
bool solveArmorPose(const Armor& armor, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                    float armor_width, float armor_height,
                    cv::Mat& rvec, cv::Mat& tvec)
{
    if(armor.image_points.size() != 4) return false;
    float half_w = armor_width / 2.0f;
    float half_h = armor_height / 2.0f;
    std::vector<cv::Point3f> object_points;
    // 对应图像点顺序：左上, 左下, 右上, 右下
    //y轴向下，x轴向右
    object_points.push_back(cv::Point3f(-half_w, -half_h, 0.0f));
    object_points.push_back(cv::Point3f(-half_w, half_h, 0.0f));
    object_points.push_back(cv::Point3f(half_w, -half_h, 0.0f));
    object_points.push_back(cv::Point3f(half_w, half_h, 0.0f));

    // 将 image_points 转为 Point2f 确保类型一致
    std::vector<cv::Point2f> img_pts = armor.image_points;

    bool ok = cv::solvePnP(object_points, img_pts, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    return ok;
}
