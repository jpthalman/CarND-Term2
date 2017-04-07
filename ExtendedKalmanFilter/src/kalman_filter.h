//
// Created by japata on 3/27/17.
//

#ifndef EXTENDEDKALMANFILTER_KALMAN_FILTER_H
#define EXTENDEDKALMANFILTER_KALMAN_FILTER_H

#include "Eigen/Dense"
#include "sensor_data_packet.h"


class KalmanFilter {
public:
    friend class FusionEKF;

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
    void Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);

    /***/
    void Update(
            const Eigen::VectorXd &z,
            const Eigen::MatrixXd &H,
            const Eigen::MatrixXd &R,
            const SensorDataPacket::SensorType sensor_type);

private:
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    // passed as argument to predict

    // environment covariance matrix
    // passed as argument to predict

    // sensor matrix
    // passed as argument to update

    // sensor covariance matrix
    // passed as argument to update

    // identity matrix
    const Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(4, 4);

    /***/
    Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd &z);
};


#endif //EXTENDEDKALMANFILTER_KALMAN_FILTER_H
