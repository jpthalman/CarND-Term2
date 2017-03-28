//
// Created by japata on 3/27/17.
//

#include "kalman_filter.h"


using namespace Eigen;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z, const SensorDataPacket::SensorType sensor_type) {
    VectorXd y;
    if (sensor_type == SensorDataPacket::RADAR)
        y = z - H_ * CartesianToPolar(x_);
    else
        y = z - H_ * x_;

    MatrixXd Ht = H_.transpose();
    MatrixXd Si = (H_ * P_ * Ht + R_).inverse();
    MatrixXd Kp = P_ * H_ * Si;

    x_ += Kp * y;
    P_ = (I_ + Kp * H_) * P_;
}

VectorXd KalmanFilter::CartesianToPolar(const VectorXd &z) {
    double px = z[0];
    double py = z[1];
    double vx = z[2];
    double vy = z[3];

    double d = sqrt(px*px + py*py);

    Vector3d output;
    output <<   d, // rho
                atan2(py, px), // phi
                (px*vx + py*vy) / d; // rho_dot
    return output;
}
