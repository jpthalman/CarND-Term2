//
// Created by japata on 3/27/17.
//

#include "kalman_filter.h"
#include <iostream>


using namespace Eigen;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;

    std::cout << "::PREDICT::" << std::endl;
    std::cout << "State:" << std::endl << x_ << std::endl << std::endl;
    std::cout << "Covariance:" << std::endl << P_ << std::endl << std::endl;
}

void KalmanFilter::Update(const Eigen::VectorXd &z, const SensorDataPacket::SensorType sensor_type) {
    VectorXd y;
    if (sensor_type == SensorDataPacket::RADAR)
        y = z - CartesianToPolar(x_);
    else
        y = z - H_ * x_;  // 2x1

    MatrixXd Ht = H_.transpose();  // 4x2
    MatrixXd S = H_ * P_ * Ht + R_;  // 2x2
    MatrixXd Kp = P_ * Ht * S.inverse();  // 4x2

    x_ += Kp * y;  // 4x1
    P_ = (I_ + Kp * H_) * P_;  // 4x4

    std::cout << "::UPDATE::" << std::endl;
    std::cout << "State:" << std::endl << x_ << std::endl << std::endl;
    std::cout << "Covariance:" << std::endl << P_ << std::endl << std::endl;
}

VectorXd KalmanFilter::CartesianToPolar(const VectorXd &z) {
    double px = z(0);
    double py = z(1);
    double vx = z(2);
    double vy = z(3);

    Vector3d output = Vector3d::Zero();
    if (fabs(px) + fabs(py) < 1e-5)
        return output;

    double d = sqrt(px*px + py*py);
    output <<   d, // rho
                atan2(py, px), // phi
                (px*vx + py*vy) / d; // rho_dot
    return output;
}
