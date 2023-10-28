#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>

void _dfs(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Vec4i>& hierachy, cv::Mat& drawer, int index, int depth);
bool judge_contour(std::vector<cv::Point>& contour);
class Pack{
public: 
    cv::Mat input;
    cv::Mat output;
    Pack(cv::Mat input, cv::Mat output): input(input), output(output){}
};
void thresh_binary(int value, void* pack) {
    Pack *data = (Pack*) pack;
    cv::threshold(data->input, data->output, value, 255, cv::THRESH_BINARY);
    cv::imshow("apple", data->output);
}

void hsv_extract(int value, void* pack) {
    Pack *data = (Pack*) pack;
    cv::inRange(data->input, cv::Scalar(value, 43, 46), cv::Scalar(155, 255, 255), data->output);
    cv::imshow("hsv_mask", 255 - data->output);
}


int main() {
    cv::Mat apple = cv::imread("../apple.png");
    cv::namedWindow("apple");
    cv::namedWindow("hsv_mask");
    cv::Mat channels[3];
    cv::split(apple, channels);
    channels[2] -= channels[1];
    cv::Mat res;
    int thresh = 28;
    cv::threshold(channels[2], res, thresh, 255, cv::THRESH_BINARY);
    cv::imshow("res", res);
    cv::Mat morph;
    cv::morphologyEx(res, morph, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 30)));
    cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(60, 60)));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(morph, contours, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    _dfs(contours, hierachy, apple, 0, 0);
    cv::imwrite("../apple_target.png", apple);
    return 0;
}

void _dfs(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Vec4i>& hierachy, cv::Mat& drawer, int index, int depth) {
    if (index == -1) return;
    if (judge_contour(contours[index])) {
        cv::drawContours(drawer, contours, index, {255, 255, 255}, 3);
        cv::Rect rect = cv::boundingRect(contours[index]);
        cv::rectangle(drawer, rect, {255, 255, 255}, 3);
    }
    for (int i = hierachy[index][3]; i != -1; i = hierachy[i][0]) {
        _dfs(contours, hierachy, drawer, i, depth + 1);
    }
}

bool judge_contour(std::vector<cv::Point>& contour) {
    return true;
}
