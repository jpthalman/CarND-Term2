//
// Created by jacob on 3/28/2017.
//

#include "FusionEKF.h"
#include "tools.h"

using namespace Eigen;


FusionEKF::FusionEKF() {
    is_initialized_ = false;

    // initializing matrices
    F_ = MatrixXd::Identity(4, 4);
    Q_ = Matrix4d::Zero();

    // sensor matrix - laser
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // sensor matrix - radar
    H_radar_ = MatrixXd(3, 4);
    // dynamically calculated

    // sensor covariance matrix - laser
    R_laser_ = MatrixXd::Identity(2, 2) * 0.0225;

    // sensor covariance matrix - radar
    R_radar_ = MatrixXd::Identity(3, 3) * 0.09;
    R_radar_(1, 1) = 0.0009;

    // environment noise
    noise_ax_ = 9.0;
    noise_ay_ = 9.0;
}


FusionEKF::~FusionEKF() {}


VectorXd FusionEKF::ProcessMeasurement(const SensorDataPacket &data) {
    /************************************************************************************
     *                                      Initialize
     ************************************************************************************/

    if (!is_initialized_) {
        if (data.sensor_type == SensorDataPacket::LIDAR) {
            kf_.x_ << data.values(0), data.values(1), 0., 0.;

            // set velocity variances to 1000
            kf_.P_(2, 2) = kf_.P_(3, 3) = 1000.0;
        }
        else if (data.sensor_type == SensorDataPacket::RADAR) {
            kf_.x_ = polar_to_cartesian(data.values);

            // set velocity variances to 10
            kf_.P_(2, 2) = kf_.P_(3, 3) = 10.0;
        }

        previous_timestamp_ = data.timestamp;
        is_initialized_ = true;
        return kf_.x_;
    }

    /************************************************************************************
     *                                      Predict
     ************************************************************************************/

    // timestep expressed in seconds
    double dt = (data.timestamp - previous_timestamp_) / 1e6;

    // pre-compute values for covariance matrix
    double dt2 = dt * dt;
    double dt3 = dt * dt2;
    double dt4 = dt * dt3;

    // update state transition matrix with time-step
    // 1  0  dt 0
    // 0  1  0  dt
    // 0  0  1  0
    // 0  0  0  1
    F_(0, 2) = F_(1, 3) = dt;

    // update non-zero state covariance values
    Q_(0, 0) = dt4/4.0*noise_ax_; Q_(0, 2) = dt3/2.0*noise_ax_;
    Q_(1, 1) = dt4/4.0*noise_ay_; Q_(1, 3) = dt3/2.0*noise_ay_;
    Q_(2, 0) = dt3/2.0*noise_ax_; Q_(2, 2) = dt2*noise_ax_;
    Q_(3, 1) = dt3/2.0*noise_ay_; Q_(3, 3) = dt2*noise_ay_;

    kf_.Predict(F_, Q_);

    /************************************************************************************
     *                                      Update
     ************************************************************************************/

    H_ = (data.sensor_type == SensorDataPacket::LIDAR ? H_laser_ : calculate_jacobian(data.values));
    R_ = (data.sensor_type == SensorDataPacket::LIDAR ? R_laser_ : R_radar_);

    kf_.Update(data.values, H_, R_, data.sensor_type);
    return kf_.x_;
}


VectorXd FusionEKF::GetCurrentState() {
    return kf_.x_;
}
