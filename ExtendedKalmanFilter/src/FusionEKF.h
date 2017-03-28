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
    Eigen::VectorXd ProcessMeasurement(const Eigen::VectorXd &z);

private:
    // Kalman filter predict and update math lives here
    KalmanFilter kf_;

    // checks whether the EKF has seen the first measurement or not
    bool is_initialized_;

    // previous timestamp. Used to measure delta-t
    long long previous_timestamp_;

    // sensor matrices
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd H_radar_;

    // sensor covariance matrices
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
};

#endif //EXTENDEDKALMANFILTER_FUSIONEKF_H
