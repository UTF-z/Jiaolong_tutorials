#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "big_armor_scale.hpp"
#include <string>
using namespace std;


vector<double> str2quad(string qstr) {
    istringstream iss(qstr);
    string s;
    vector<double> q(4);
    while(iss >> s) {
        if (s[0] == '+') {
            continue;
        }
        if (s.back() == 'i') {
            s.pop_back();
            q[0] = stod(s);
        }
        if (s.back() == 'j') {
            s.pop_back();
            q[1] = stod(s);
        }
        if (s.back() == 'k') {
            s.pop_back();
            q[2] = stod(s);
        } else {
            q[3] = stod(s);
        }
    }
    return q;
}

int main() {
    // Read in f mat and c mat
    cv::FileStorage readin = cv::FileStorage("../resource/f_mat_and_c_mat.yml", cv::FileStorage::READ);
    cv::Mat f_mat;
    readin["F"] >> f_mat;
    cv::Mat c_mat;
    readin["C"] >> c_mat;

    // Read in q. ATTENTION ! q -> c2w: imu_coord = c2w * camera_coord.
    ifstream ifs("../source/imu_and_armor.txt", ios::in);
    string s;
    getline(ifs, s);
    getline(ifs, s);
    vector<double> qvec = str2quad(s);
    Eigen::Quaternion<double> q(qvec.data());
    Eigen::Matrix3d c2i = q.matrix();

    // Read in pps
    getline(ifs, s);
    double x, y;
    vector<cv::Point2d> pps;
    for (int i = 0; i < 4; ++i) {
        ifs >> x >> y;
        pps.push_back(cv::Point2d(x, y));
    }

    // Get rvec and tvec. ATTENTION! rvec -> w2c, tvec -> w2c
    cv::Vec3d rvec, tvec;
    cv::solvePnP(PW_BIG, pps, f_mat, c_mat, rvec, tvec);
    cv::Mat w2c;
    cv::Rodrigues(rvec, w2c);

    // Find center in camera space: camera_coord = w2c * world_coord + tvec
    cv::Vec3d center = {0, 0, 0};
    for (int i = 0; i < PW_BIG.size(); i++) {
        center = center + cv::Vec3d(PW_BIG[i].x, PW_BIG[i].y, PW_BIG[i].z);
    }
    center = center / double(PW_BIG.size());
    cv::Mat center_camera = w2c * cv::Mat(center) + cv::Mat(tvec);

    // Find center in IMU center: imu_coord = c2i * camera_coord
    Eigen::Vector3d center_imu = Eigen::Vector3d(center_camera.at<double>(0, 0), center_camera.at<double>(1, 0), center_camera.at<double>(2, 0));
    Eigen::Vector3d res = c2i * center_imu;
    cout << "res: " << res.transpose() << endl;
    return 0;
}