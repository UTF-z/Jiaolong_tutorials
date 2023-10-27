#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
#include<vector>

std::string name_generator(int a, std::string prefix = "") {
    return "00" + std::to_string(a) + prefix + ".jpg";
}

bool judge_contour(std::vector<cv::Point> &contour) {
    if (!cv::isContourConvex(contour))
        return false;
    if (cv::arcLength(contour, true) < 100)
        return false;
    return true;
}

std::string img_dir = "../plates";
std::string img_target_dir = "../plates_target";
int main() {
    for (int i = 1; i <= 5; ++i) {
        std::string name = name_generator(i);
        cv::Mat img = cv::imread(img_dir + '/' + name);
        cv::Mat channels[3];
        cv::split(img, channels);
        channels[0] -= channels[1];
        cv::Mat mask;
        cv::threshold(channels[0], mask, 0, 255, cv::THRESH_OTSU);
        cv::Mat morph;
        cv::morphologyEx(mask, morph, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {50, 50}));
        cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));
        //cv::morphologyEx(morph, morph, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
        cv::imshow("morph", morph);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierachy;
        cv::findContours(morph, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); ++i) {
            std::vector<cv::Point> poly;
            cv::approxPolyDP(contours[i], poly, 10, true);
            std::vector<std::vector<cv::Point>> polys = {poly};
            if (judge_contour(poly)) {
                cv::drawContours(img, polys, 0, {0, 0, 255}, 3);
            }
        }
        std::string target_name = name_generator(i, "target");
        cv::imwrite(img_target_dir + '/' + target_name, img);
    }
    return 0;
}