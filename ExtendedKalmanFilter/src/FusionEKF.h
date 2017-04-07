//
// Created by jacob on 3/28/2017.
//

#ifndef EXTENDEDKALMANFILTER_FUSIONEKF_H
#define EXTENDEDKALMANFILTER_FUSIONEKF_H

#include "kalman_filter.h"
#include "tools.h"


class FusionEKF {
public:
    // constructor
    FusionEKF();

    // destructor
    virtual ~FusionEKF();

    /**
     * Run the complete flow of the filter with this function.
     * */
    Eigen::VectorXd ProcessMeasurement(const SensorDataPacket &data);

    /**
     * Get the current state.
     * */
    Eigen::VectorXd GetCurrentState();

private:
    // Kalman filter predict and update math lives here
    KalmanFilter kf_;

    // checks whether the EKF has seen the first measurement or not
    bool is_initialized_;

    // previous timestamp. Used to measure delta-t
    long long previous_timestamp_;

    // temporary matrices to pass to the kalman filter math
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;

    // sensor matrices
    Eigen::MatrixXd H_laser_;

    // sensor covariance matrices
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;

    // environment noise values
    float noise_ax_;
    float noise_ay_;
};

#endif //EXTENDEDKALMANFILTER_FUSIONEKF_H
