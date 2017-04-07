//
// Created by japata on 3/27/17.
//

#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using namespace Eigen;


KalmanFilter::KalmanFilter() {
    x_ = Vector4d::Zero();
    P_ = MatrixXd::Identity(4, 4);
}


KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q) {
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;

    std::cout << "::PREDICT::" << std::endl;
    std::cout << "State:" << std::endl << x_ << std::endl << std::endl;
    std::cout << "Covariance:" << std::endl << P_ << std::endl << std::endl;
}


void KalmanFilter::Update(
        const Eigen::VectorXd &z,
        const Eigen::MatrixXd &H,
        const Eigen::MatrixXd &R,
        const SensorDataPacket::SensorType sensor_type) {

    VectorXd y;
    if (sensor_type == SensorDataPacket::RADAR)
        y = z - cartesian_to_polar(x_);
    else
        y = z - H * x_;  // 2x1

    MatrixXd Ht = H.transpose();  // 4x2
    MatrixXd S = H * P_ * Ht + R;  // 2x2
    MatrixXd Kp = P_ * Ht * S.inverse();  // 4x2

    x_ += Kp * y;  // 4x1
    P_ = (I_ - Kp * H) * P_;  // 4x4

    std::cout << "::UPDATE::" << std::endl;
    std::cout << "State:" << std::endl << x_ << std::endl << std::endl;
    std::cout << "Covariance:" << std::endl << P_ << std::endl << std::endl;
}
