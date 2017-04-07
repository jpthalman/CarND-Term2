//
// Created by jacob on 3/28/2017.
//

#include "FusionEKF.h"
#include "tools.h"

using namespace Eigen;

FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    H_laser_ = MatrixXd(2, 4);
    H_radar_ = MatrixXd(3, 4);
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    kf_.x_ = Vector4d::Zero();
    kf_.F_ = MatrixXd::Identity(4, 4);
    kf_.Q_ = Matrix4d::Zero();
    kf_.P_ = MatrixXd::Identity(4, 4);

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

    // environment noise
    noise_ax = 9.0;
    noise_ay = 9.0;
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
            double rho = data.values(0);
            double phi = data.values(1);
            double rho_dot = data.values(2);

            kf_.x_ <<   rho * sin(phi), // px
                        rho * cos(phi), // py
                        rho_dot * sin(phi), // vx
                        rho_dot * cos(phi); //vy
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
    kf_.F_(0, 2) = kf_.F_(1, 3) = dt;

    // update non-zero state covariance values
    kf_.Q_(0, 0) = dt4/4.0*noise_ax; kf_.Q_(0, 2) = dt3/2.0*noise_ax;
    kf_.Q_(1, 1) = dt4/4.0*noise_ay; kf_.Q_(1, 3) = dt3/2.0*noise_ay;
    kf_.Q_(2, 0) = dt3/2.0*noise_ax; kf_.Q_(2, 2) = dt2*noise_ax;
    kf_.Q_(3, 1) = dt3/2.0*noise_ay; kf_.Q_(3, 3) = dt2*noise_ay;

    kf_.Predict();

    /************************************************************************************
     *                                      Update
     ************************************************************************************/

    kf_.H_ = (data.sensor_type == SensorDataPacket::LIDAR ? H_laser_ : calculate_jacobian(data.values));
    kf_.R_ = (data.sensor_type == SensorDataPacket::LIDAR ? R_laser_ : R_radar_);

    kf_.Update(data.values, data.sensor_type);
    return kf_.x_;
}


VectorXd FusionEKF::GetCurrentState() {
    return kf_.x_;
}
