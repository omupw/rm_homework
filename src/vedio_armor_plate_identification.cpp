#include <iostream>
#include <opencv2/opencv.hpp>

//编译命令：g++ -o ../bin/vedio_armor_plate_identification vedio_armor_plate_identification.cpp `pkg-config --cflags --libs opencv4`

void drawRotatedRect(cv::Mat& image, const cv::RotatedRect& rotatedRect, const cv::Scalar& color, int thickness) {
    cv::Point2f vertices[4];
    rotatedRect.points(vertices);
    for (int j = 0; j < 4; j++) {
        cv::line(image, vertices[j], vertices[(j + 1) % 4], color, thickness);
    }
}

int main() {
    // Load the video
    cv::VideoCapture cap("../data/armor/avi.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error opening video stream!" << std::endl;
        return -1;
    }
    while (true) {
        cv::Mat frame;
        cap >> frame; // Capture frame-by-frame
        if (frame.empty()) {
            break; // Exit if the video has ended
        }
        // Display the resulting frame
        cv::Mat hsvImage;
        cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);
        // Define the lower and upper bounds for the color of the armor plate
        //灯条为红白色，HSV空间中红色的范围大约是H: 0-10, S: 150-255, V: 50-255
        cv::Scalar lowerBound(0, 150, 50); 
        cv::Scalar upperBound(10, 255, 255); 
        // Create a mask based on the defined color range
        cv::Mat mask;
        cv::inRange(hsvImage, lowerBound, upperBound, mask);
        // Find contours in the mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // 使用rotatedRect来获取更准确的矩形框
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) > 25) { // Filter small contours
                cv::RotatedRect rotatedBox = cv::minAreaRect(contour);
                //灯条为竖向矩形，即rotated的角度的绝对值大于30度,且宽高比在一定范围内
                double aspectRatio = std::max(rotatedBox.size.width, rotatedBox.size.height) / std::min(rotatedBox.size.width, rotatedBox.size.height);       
                if (std::abs(rotatedBox.angle) > 30 && aspectRatio > 2) { // Filter based on the angle of the rectangle
                    drawRotatedRect(frame, rotatedBox, cv::Scalar(0, 255, 0), 2);
                }
            }       
        }
        // Display the result
        cv::imshow("Video Stream", frame);
        // Press 'q' to exit
        if (cv::waitKey(30) == 'q') {
            break;
        }
    }
    // Release the video capture object and close all OpenCV windows
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
