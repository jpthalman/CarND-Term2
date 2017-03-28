//
// Created by japata on 3/27/17.
//

#ifndef EXTENDEDKALMANFILTER_KALMAN_FILTER_H
#define EXTENDEDKALMANFILTER_KALMAN_FILTER_H

#include "Eigen/Dense"
#include "sensor_data_packet.h"


class KalmanFilter {
public:
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // environment covariance matrix
    Eigen::MatrixXd Q_;

    // sensor matrix
    Eigen::MatrixXd H_;

    // sensor covariance matrix
    Eigen::MatrixXd R_;

    /**
     * Constructor
     * */
    KalmanFilter();

    /**
     * Destructor
     * */
    virtual ~KalmanFilter();

    /**
     * Predict state k+1 based upon the current state vector (x_), state covariance (P_), and environment noise (Q_).
     * */
    void Predict();

    /***/
    void Update(const Eigen::VectorXd &z, const SensorDataPacket::SensorType sensor_type);

private:
    // identity matrix
    const Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(2, 2);

    /***/
    Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd &z);
};


#endif //EXTENDEDKALMANFILTER_KALMAN_FILTER_H
