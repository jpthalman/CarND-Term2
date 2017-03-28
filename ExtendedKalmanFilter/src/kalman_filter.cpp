//
// Created by japata on 3/27/17.
//

#include "kalman_filter.h"


using namespace Eigen;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    x_ = H_ * x_;
    P_ = H_ * P_ * H_.transpose() + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z, const SensorDataPacket::SensorType sensor_type) {
    // TODO
}
