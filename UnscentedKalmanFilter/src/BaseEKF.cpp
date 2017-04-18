//
// Created by japata on 4/17/17.
//

#include "BaseEKF.h"
#include "tools.h"

using namespace Eigen;

BaseEKF::BaseEKF(int n_states, std::vector<double> noise_stdevs, double lambda) :
        n_states_(n_states),
        n_aug_states_(n_states + noise_stdevs.size()),
        lambda_(lambda),
        x_(VectorXd(n_states)),
        P_(MatrixXd::Identity(n_states, n_states)),
        Q_(MatrixXd(noise_stdevs.size(), noise_stdevs.size())),
        sigma_points_(MatrixXd(n_aug_states_, 2 * n_aug_states_ + 1)),
        weights_(VectorXd(2 * n_states + 1)),
        is_initialized_(false)
{
    // instantiate the process noise matrix
    Q_.fill(0.0);
    for (int i = 0; i < noise_stdevs.size(); ++i)
        Q_(i, i) = pow(noise_stdevs[i], 2);

    // instantiate the weights for the prediction step
    weights_.fill(0.5 / (lambda_ + n_aug_states_));
    weights_(0) = lambda_ / (lambda_ + n_aug_states_);
}

void BaseEKF::ProcessMeasurement(SensorDataPacket &data) {
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
            x_(3) = atan2(x_(1), x_(0));  // yaw
            // TODO: estimate yaw rate with velocities
        }

        prev_timestamp_ = data.timestamp;
        is_initialized_ = true;
        return;
    }

    /**********************************************************************************
     *                                     Predict
     **********************************************************************************/

    // calculate the time step in seconds since the last prediction
    double dt = (data.timestamp - prev_timestamp_) / 1e6;

    // create augmented state and covariance to include noise
    VectorXd x_aug = VectorXd(n_aug_states_);
    x_aug << x_, 0, 0;

    int n_noise_coeff = n_aug_states_ - n_states_;
    MatrixXd P_aug = MatrixXd(n_aug_states_, n_aug_states_);

    P_aug.topLeftCorner(n_states_, n_states_) = P_;
    P_aug.bottomRightCorner(n_noise_coeff, n_noise_coeff) = Q_;

    // create the initial sigma points
    GenerateSigmaPoints(x_aug, P_aug);

    // transform the sigma points into the new space
    PredictSigmaPoints(sigma_points_, dt);

    // predict the new mean and covariance with the new sigma points
    GetMeanAndCovariance();

    /**********************************************************************************
     *                                      Update
     **********************************************************************************/

    // TODO

    // store the UKF prediction in the referenced data packet
    data.predictions = x_;
    return;
}

void BaseEKF::GenerateSigmaPoints(Eigen::VectorXd &x, Eigen::MatrixXd &P) {
    double scaling_factor = sqrt(n_aug_states_ + lambda_);
    MatrixXd P_sqrt = P.llt().matrixL();

    sigma_points_ = MatrixXd(n_aug_states_, 2 * n_aug_states_ + 1);
    sigma_points_ << x, (scaling_factor * P_sqrt).colwise() + x, (-scaling_factor * P_sqrt).colwise() + x;
}

void BaseEKF::GetMeanAndCovariance() {
    x_ = sigma_points_ * weights_;

    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_states_ + 1; ++i) {
        VectorXd diff = sigma_points_.col(i) - x_;

        // normalize the angles
        while (diff(3) > M_PI)
            diff(3) -= 2. * M_PI;
        while (diff(3) < M_PI)
            diff(3) += 2. * M_PI;

        P_ += weights_(i) * diff * diff.transpose();
    }
}
