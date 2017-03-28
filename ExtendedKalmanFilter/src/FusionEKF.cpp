//
// Created by jacob on 3/28/2017.
//

#include "FusionEKF.h"

using namespace Eigen;

FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    H_laser_ = MatrixXd(2, 4);
    H_radar_ = MatrixXd(3, 4);
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);

    // sensor matrix - laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // sensor matrix - radar
    // dynamically calculated

    // sensor covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    // sensor covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
}

FusionEKF::~FusionEKF() {}

VectorXd FusionEKF::ProcessMeasurement(const VectorXd &z) {
    // TODO
}
