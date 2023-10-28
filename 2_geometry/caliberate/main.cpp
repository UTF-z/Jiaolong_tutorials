#include<fstream>
#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include<string>
#include<vector>
using namespace std;

string DIR = "../chess/";
ofstream data_out("../caliberate_res.txt", ios::out);

int main() {
    int row_num = 9, col_num = 6;
    cv::Size size(row_num, col_num);
    cv::Size square_size(10, 10);
    cv::Size img_size;
    vector<vector<cv::Point2f>> img_points;
    vector<vector<cv::Point3f>> obj_points;
    vector<cv::Point2f> corners;
    vector<cv::Point3f> obj_point;
    cv::Mat gray;
    int success = 0;

    // Pay attention to the push order, consistent with corners.
    for (int i = 0; i < col_num; i ++) {
        for (int j = 0; j < row_num; ++j) {
            cv::Point3f pt;
            pt.x = i * square_size.width;
            pt.y = j * square_size.height;
            pt.z = 0;
            obj_point.push_back(pt);
        }
    }

    for (int i = 0; i <= 40; ++i) {
        string img_name = DIR + to_string(i) + ".jpg";
        cv::Mat img = cv::imread(img_name);
        img_size.width = img.cols;
        img_size.height = img.rows;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        bool found = cv::findChessboardCorners(gray, size, corners);
        bool subpix = cv::find4QuadCornerSubpix(gray, corners, {5, 5});
        if (found && subpix) {
            success++;
            img_points.push_back(corners);
            cv::drawChessboardCorners(img, size, corners, found);
            cv::imshow("corners", img);
            cv::waitKey(0);
        }
    }
    for (int i = 0; i < success; ++i) {
        obj_points.push_back(obj_point);
    }
    cv::Mat camera_mat, distort_coeff;
    vector<cv::Mat> rvecs, tvecs;
    double error = cv::calibrateCamera(obj_points, img_points, img_size, camera_mat, distort_coeff, rvecs, tvecs);
    data_out << "camera mat: " << endl << camera_mat << endl;
    data_out << "distortion coeff: " << endl << distort_coeff << endl;
}