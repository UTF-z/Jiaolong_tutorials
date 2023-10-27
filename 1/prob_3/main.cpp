#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

bool judge(std::vector<cv::Point> &contour) {
    if (cv::arcLength(contour, true) < 150) {
        return false;
    }
    return true;
}

int main() {
    cv::VideoCapture reader = cv::VideoCapture("../armor.mp4");
    cv::Mat frame;
    cv::Mat gray_frame;
    cv::Mat hsv_frame;
    cv::Mat hsv_mask1;
    cv::Mat hsv_mask2;
    cv::Mat thresh_mask;
    cv::Mat morph_mask;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    int thresh = 200;
    reader.read(frame);
    cv::VideoWriter writer = cv::VideoWriter("../armor_target.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(frame.cols, frame.rows), true);
    do {
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_frame, cv::Scalar(0, 0, 46), cv::Scalar(34, 255, 255), hsv_mask1);
        cv::inRange(hsv_frame, cv::Scalar(156, 0, 46), cv::Scalar(180, 255, 255), hsv_mask2);
        cv::threshold(gray_frame, thresh_mask, thresh, 255, cv::THRESH_BINARY);
        thresh_mask = thresh_mask & (hsv_mask1 | hsv_mask2);
        cv::morphologyEx(thresh_mask, morph_mask, cv::MORPH_ERODE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 20)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(100, 10)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 15)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 10)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 10)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 200)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 200)));
        cv::morphologyEx(morph_mask, morph_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 15)));
        cv::findContours(morph_mask, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); i ++) {
            if (judge(contours[i])) {
                cv::Rect bb = cv::boundingRect(contours[i]);
                cv::rectangle(frame, bb, {0, 0, 255}, 3);
                cv::circle(frame, 0.5 * (cv::Point2i(bb.x, bb.y) + bb.br()), 10, {0, 255, 255}, -1);
            }
        }
        writer << frame;
    } while (reader.read(frame));
    return 0;
}