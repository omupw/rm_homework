#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

int main() {
    // 1. 批量读取标定图片
    std::vector<cv::String> imagePaths;
    cv::glob("../data/calibration/*.jpg", imagePaths);
    
    if (imagePaths.empty()) {
        std::cerr << "未找到标定图片！" << std::endl;
        return -1;
    }
    
    // 2. 棋盘格参数
    cv::Size boardSize(9, 6);        // 内角点：9列×6行
    float squareSize = 22.5f;        // 每个格子22.5mm
    
    // 3. 生成世界坐标（单位：mm）
    std::vector<cv::Point3f> worldCorners;
    for (int y = 0; y < boardSize.height; y++) {
        for (int x = 0; x < boardSize.width; x++) {
            worldCorners.push_back(cv::Point3f(
                x * squareSize,  // X: mm
                y * squareSize,  // Y: mm  
                0.0f             // Z: mm (棋盘格平面)
            ));
        }
    }
    
    // 4. 检测每张图的角点
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    cv::Size imageSize;
    
    for (size_t i = 0; i < imagePaths.size(); i++) {
        cv::Mat image = cv::imread(imagePaths[i], cv::IMREAD_GRAYSCALE);
        if (image.empty()) continue;
        
        imageSize = image.size();
        
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            image, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
        );
        
        if (found) {
            // 亚像素精确化
            cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            
            imagePoints.push_back(corners);
            objectPoints.push_back(worldCorners);
            
            std::cout << "[" << i+1 << "/" << imagePaths.size() 
                      << "] 成功检测: " << imagePaths[i] << std::endl;
        } else {
            std::cout << "[" << i+1 << "/" << imagePaths.size() 
                      << "] 未检测到角点: " << imagePaths[i] << std::endl;
        }
    }
    
    // 5. 检查数据
    if (imagePoints.size() < 3) {
        std::cerr << "有效图片不足3张，无法标定！" << std::endl;
        return -1;
    }
    
    std::cout << "\n使用 " << imagePoints.size() << " 张图片进行标定..." << std::endl;
    
    // 6. 执行标定（最简单的5参数调用）
    cv::Mat cameraMatrix, distCoeffs;
    double error = cv::calibrateCamera(
        objectPoints,   // 世界坐标（单位：mm）
        imagePoints,    // 图像坐标（像素）
        imageSize,      // 图像尺寸
        cameraMatrix,   // 输出：内参矩阵
        distCoeffs,     // 输出：畸变系数 (k1,k2,p1,p2,k3)
        cv::noArray(),  // 标定选项（默认即可）
        cv::noArray(),  // 标定选项（默认即可）
    );
    
    // 7. 输出结果
    std::cout << "\n========== 标定结果 ==========" << std::endl;
    std::cout << "使用图片: " << imagePoints.size() << " 张" << std::endl;
    std::cout << "图像尺寸: " << imageSize << std::endl;
    std::cout << "棋盘格尺寸: " << boardSize << " (内角点)" << std::endl;
    std::cout << "格子大小: " << squareSize << " mm" << std::endl;
    std::cout << "重投影误差: " << error << " 像素" << std::endl;
    
    std::cout << "\n内参矩阵:" << std::endl;
    std::cout << cameraMatrix << std::endl;
    
    std::cout << "\n畸变系数 (k1,k2,p1,p2,k3):" << std::endl;
    std::cout << distCoeffs.t() << std::endl;
    
    return 0;
}