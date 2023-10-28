#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
namespace py = pybind11;
using namespace std;

class KalmanFilter {
    Eigen::Matrix<double, 2, 1> x;
    Eigen::Matrix<double, 2, 2> A;
    Eigen::Matrix<double, 2, 2> P;
    Eigen::Matrix<double, 2, 2> R;
    Eigen::Matrix<double, 1, 2> C;
    Eigen::Matrix<double, 2, 1> K;
    Eigen::Matrix<double, 1, 1> Q;
public:
    KalmanFilter(
        Eigen::Matrix<double, 2, 1> x,
        Eigen::Matrix<double, 2, 2> A,
        Eigen::Matrix<double, 2, 2> P, 
        Eigen::Matrix<double, 2, 2> R, 
        Eigen::Matrix<double, 1, 2> C, 
        Eigen::Matrix<double, 1, 1> Q
    ) {
        this->A = A;
        this->P = P;
        this->R = R;
        this->C = C;
        this->Q = Q;
        this->x = x;
    }

    double update(Eigen::Matrix<double, 1, 1> z) {
        Eigen::Matrix<double, -1, 1> nx = A * x;
        P = A * P * A.transpose() + R;
        K = P * C.transpose() * (C * P * C.transpose() + Q).inverse();
        P = (Eigen::Matrix<double, 2, 2>::Identity() - K * C) * P;
        x = nx + K * (z - C * nx);
        return x(0, 0);
    }

    double predict(int t) {
        double current_z = x(0, 0);
        double current_v = x(1, 0);
        return current_z + current_v * t;
    }
};

PYBIND11_MODULE(Filters, m) {
    py::class_<KalmanFilter> filter(m, "KalmanFilter");
    filter.def(py::init<Eigen::Matrix<double, 2, 1>, Eigen::Matrix<double, 2, 2>, Eigen::Matrix<double, 2, 2>, Eigen::Matrix<double, 2, 2>, Eigen::Matrix<double, 1, 2>, Eigen::Matrix<double, 1, 1>>());
    filter.def("update", &KalmanFilter::update);
    filter.def("predict", &KalmanFilter::predict);
}
