//
// Created by jaket on 4/13/2017.
//

#include "unscented_kalman_filter.h"

using namespace Eigen;


UnscentedKalmanFilter::UnscentedKalmanFilter() {
    // TODO
}

UnscentedKalmanFilter::~UnscentedKalmanFilter() {
    // TODO
}

VectorXd UnscentedKalmanFilter::ProcessMeasurement(SensorDataPacket &data) {
    // TODO
    return x_;
}
