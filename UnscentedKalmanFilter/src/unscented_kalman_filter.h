//
// Created by jaket on 4/13/2017.
//

#ifndef UNSCENTEDKALMANFILTER_UNSCENTED_KALMAN_FILTER_H
#define UNSCENTEDKALMANFILTER_UNSCENTED_KALMAN_FILTER_H

#include "sensor_data_packet.h"
#include "Eigen/Dense"


class UnscentedKalmanFilter {
public:
    UnscentedKalmanFilter();
    virtual ~UnscentedKalmanFilter();

    Eigen::VectorXd GetCurrentState() const { return x_; }
    Eigen::MatrixXd GetCurrentCovariance() const { return P_; }

    Eigen::VectorXd ProcessMeasurement(SensorDataPacket &data);

private:
    // state vector
    Eigen::VectorXd x_;

    // state covariance
    Eigen::MatrixXd P_;
};


#endif //UNSCENTEDKALMANFILTER_UNSCENTED_KALMAN_FILTER_H
