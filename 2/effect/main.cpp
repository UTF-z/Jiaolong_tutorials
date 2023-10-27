#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace std;

Eigen::Matrix<double, 3, 4> get_w2p(Eigen::Quaternion<double> q, Eigen::Vector3d cam_T, const Eigen::Matrix<double, 3, 4> &inner_param) {
    Eigen::Matrix<double, 4, 4> c2w;
    Eigen::Matrix<double, 3, 3> rotation_matrix = q.toRotationMatrix();
    c2w.block(0, 0, 3, 3) = rotation_matrix;
    c2w.block(0, 3, 3, 1) = cam_T;
    c2w(3, 3) = 1;
    Eigen::Matrix<double, 4, 4> w2c;
    w2c.block(0, 0, 3, 3) = rotation_matrix.transpose();
    w2c.block(0, 3, 3, 1) = -rotation_matrix.transpose() * cam_T;
    w2c(3, 3) = 1;
    return inner_param * w2c;
}

Eigen::Matrix<double, 3, 4> interpolate_w2p(Eigen::Quaternion<double> q0, Eigen::Quaternion<double> q1, Eigen::Vector3d center, Eigen::Vector3d r1, Eigen::Vector3d r2, const Eigen::Matrix<double, 3, 4> &innner_param, double t) {
    Eigen::Quaternion<double> q = q0.slerp(t, q1);
    Eigen::Vector3d camera_T = center + r1 * sin(M_PI * t - M_PI / 2.0) + r2 * cos(M_PI * t - M_PI / 2.0);
    Eigen::Matrix<double, 3, 4> transform = get_w2p(q, camera_T, innner_param);
    return transform;
}

int main() {
    ifstream read_in("../points.txt", ios::in);
    Eigen::Quaternion<double> q_0(-1, 0, 0, 0);
    Eigen::Quaternion<double> q_1(-0.5, 0.5, 0.5, -0.5);
    Eigen::Matrix<double, 3, 4> inner_param;
    inner_param << 400, 0, 190, 0, 0, 400, 160, 0, 0 ,0, 1, 0;
    Eigen::Vector3d camera_T_1 = {2.0, 2.0, 2.0};
    vector<Eigen::Vector4d> wps;
    int n;
    read_in >> n;
    double x, y, z;
    Eigen::Vector4d avg_wp = {0., 0., 0., 0.};
    const double phi0 = - M_PI / 2.0, phi1 = M_PI / 2.0, phi2 = 0;
    for (int i = 0; i < n; ++i) {
        read_in >> x >> y >> z;
        Eigen::Vector4d wp = {x, y, z, 1.0};
        avg_wp += wp;
        wps.push_back(wp);
    }
    avg_wp /= n;
    Eigen::Vector3d mid_wp = (camera_T_1 + avg_wp.block(0, 0, 3, 1)) / 2.0;
    Eigen::Vector3d r1 = camera_T_1 - mid_wp;
    Eigen::Vector3d r2 = {r1(2), 0, -r1(0)};
    int frame_num = 1000;
    Eigen::VectorXd motive = Eigen::VectorXd::LinSpaced(frame_num, 0, 1);
    cv::VideoWriter video_out("../logo_effect.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 200, cv::Size(1080, 720), true);
    cv::Mat frame(cv::Size(1080, 720), CV_8UC3);
    for (int i = 0; i < motive.size(); ++i) {
        double t = motive(i);
        frame *= 0;
        Eigen::Matrix<double, 3, 4> w2p = interpolate_w2p(q_0, q_1, mid_wp, r1, r2, inner_param, t);
        for (int j = 0; j < wps.size(); ++j) {
            Eigen::Vector3d pp = w2p * wps[j];
            pp /= pp(2);
            cv::circle(frame, cv::Point2d(pp(0), pp(1)), 1, {(255 * (1 + sin(M_PI * (t - 0.5) + phi0)) / 2.0), (255 * (1 + sin(M_PI * (t - 0.5) + phi1)) / 2.0), (255 * (1 + sin(M_PI * (t - 0.5) + phi2)) / 2.0)}, 2);
        }
        video_out << frame;
    }
    for (int i = 0; i < 30; ++i) {
        video_out << frame;
    }
    return 0;
}