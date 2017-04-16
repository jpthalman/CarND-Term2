//
// Created by jaket on 4/13/2017.
//

#include "unscented_kalman_filter.h"
#include "tools.h"

using namespace Eigen;


UnscentedKalmanFilter::~UnscentedKalmanFilter() {
    // TODO
}

void UnscentedKalmanFilter::ProcessMeasurement(SensorDataPacket &data) {
    /**********************************************************************************
     *                                    Initialize
     **********************************************************************************/
    if (!is_initialized_) {
        if (data.sensor_type == SensorDataPacket::LIDAR) {
            x_.fill(0.0);
            x_(0) = data.observations(0);  // px
            x_(1) = data.observations(1);  // py
            x_(3) = atan2(x_(1), x_(0));   // yaw
        }
        else if (data.sensor_type == SensorDataPacket::RADAR) {
            VectorXd cartesian = polar_to_cartesian(data.observations);

            x_.fill(0.0);
            x_(0) = cartesian(0);  // px
            x_(1) = cartesian(1);  // py
            x_(2) = sqrt(pow(x_(0), 2) + pow(x_(1), 2));  // v
            x_(3) = atan2(x_(1), x_(0));
            // TODO: estimate yaw rate with velocities
        }

        prev_timestamp_ = data.timestamp;
        is_initialized_ = true;
    }

    /**********************************************************************************
     *                                     Predict
     **********************************************************************************/

    // create augmented mean vector including the means of the noise processes
    VectorXd x_aug;
    x_aug << x_, 0, 0;

    // TODO

    /**********************************************************************************
     *                                      Update
     **********************************************************************************/

    // TODO

    // store the UKF prediction in the referenced data packet
    data.predictions = x_;
}

void UnscentedKalmanFilter::GenerateSigmaPoints(Eigen::VectorXd x, Eigen::MatrixXd P, int lambda) {
    double scaling_factor = sqrt(n_states_ + lambda);
    MatrixXd P_sqrt = P.llt().matrixL();

    sigma_points_ << x, (scaling_factor * P_sqrt).colwise() + x, (-scaling_factor * P_sqrt).colwise() + x;
}
