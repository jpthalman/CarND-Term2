//
// Created by japata on 4/17/17.
//

#include "BaseUKF.h"
#include "tools.h"
#include "NotImplementedException.h"

using namespace Eigen;

BaseUKF::BaseUKF(int n_states,
                 std::vector<float> noise_stdevs,
                 double lambda) :
        n_states_(n_states),
        n_aug_states_(n_states + noise_stdevs.size()),
        lambda_(lambda),
        x_(VectorXd(n_states)),
        P_(MatrixXd::Identity(n_states, n_states)),
        Q_(MatrixXd(noise_stdevs.size(), noise_stdevs.size())),
        X_sigma_points_(MatrixXd(n_aug_states_, 2 * n_aug_states_ + 1)),
        weights_(VectorXd(2 * n_aug_states_ + 1)),
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

void BaseUKF::ProcessMeasurement(
        SensorDataPacket &data)
{
    /**********************************************************************************
     *                                    Initialize
     **********************************************************************************/

    if (!is_initialized_) {
        if (data.sensor_type == SensorDataPacket::LIDAR)
        {
            x_.fill(0.0);
            x_(0) = data.observations(0);  // px
            x_(1) = data.observations(1);  // py
            x_(3) = atan2(x_(1), x_(0));   // yaw
        }
        else if (data.sensor_type == SensorDataPacket::RADAR)
        {
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
    prev_timestamp_ = data.timestamp;

    // create augmented state and covariance to include noise
    VectorXd x_aug = VectorXd(n_aug_states_);
    x_aug << x_, 0, 0;

    int n_noise_coeff = n_aug_states_ - n_states_;
    MatrixXd P_aug = MatrixXd(n_aug_states_, n_aug_states_);

    P_aug.topLeftCorner(n_states_, n_states_) = P_;
    P_aug.bottomRightCorner(n_noise_coeff, n_noise_coeff) = Q_;

    // create the initial sigma points
    GenerateSigmaPoints(x_aug, P_aug);

    try {
        // predict the sigma points to t+1
        X_sigma_points_ = PredictSigmaPoints(X_sigma_points_, dt);

        // predict the new mean and covariance with the new sigma points
        ProcessSpaceMeanAndCovariance(X_sigma_points_, x_, P_);
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }


    /**********************************************************************************
     *                                      Update
     **********************************************************************************/


    try {
        // transform sigma points into the measurement space
        Z_sigma_points_ = SigmaPointsToMeasurementSpace(X_sigma_points_, weights_, data.sensor_type);

        // get the mean and covariance of the new sigma points
        MeasurementSpaceMeanAndCovariance(Z_sigma_points_, data.sensor_type, z_, S_);
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    // calculate the kalman gain
    CalculateKalmanGain();

    // update the state and covariance
//    std::cout << K_.rows() << ' ' << K_.cols() << std::endl
//              << data.observations.size() << std::endl << z_.size() << std::endl;
//    std::cout << data.observations << std::endl << data.timestamp << std::endl << data.ground_truths << std::endl;
//
//    std::cout << (data.sensor_type == SensorDataPacket::RADAR ? "RADAR" : "LIDAR") << std::endl;

    x_ += K_ * (data.observations - z_);
    P_ += K_ * S_ * K_.transpose();

    // store the UKF prediction in the referenced data packet
    data.predictions = x_;
    return;
}

void BaseUKF::GenerateSigmaPoints(
        const Eigen::VectorXd &x,
        const Eigen::MatrixXd &P)
{
    double scaling_factor = sqrt(n_aug_states_ + lambda_);
    MatrixXd P_sqrt = P.llt().matrixL();

    X_sigma_points_ = MatrixXd(n_aug_states_, 2 * n_aug_states_ + 1);
    X_sigma_points_ << x, (scaling_factor * P_sqrt).colwise() + x, (-scaling_factor * P_sqrt).colwise() + x;
}

void BaseUKF::CalculateKalmanGain()
{
    MatrixXd cross_correlation = MatrixXd(x_.size(), z_.size());
    cross_correlation.fill(0.0);

    for (int i = 0; i < weights_.size(); ++i) {
        VectorXd x_diff = X_sigma_points_.col(i) - x_;
        VectorXd z_diff = Z_sigma_points_.col(i) - z_;

        cross_correlation += weights_(i) * x_diff * z_diff.transpose();
    }

    K_ = cross_correlation * S_.inverse();
}

MatrixXd BaseUKF::PredictSigmaPoints(
        const Eigen::MatrixXd &sigma_pts,
        const double delta_t)
{
    throw NotImplementedException("`PredictSigmaPoints` needs to be implemented.");
}

MatrixXd BaseUKF::SigmaPointsToMeasurementSpace(
        const Eigen::MatrixXd &sigma_pts,
        const Eigen::VectorXd &weights,
        const SensorDataPacket::SensorType sensor_type)
{
    throw NotImplementedException("`SigmaPointsToMeasurementSpace` needs to be implemented.");
}

void BaseUKF::ProcessSpaceMeanAndCovariance(
        const Eigen::MatrixXd &sigma_pts,
        Eigen::VectorXd &mean,
        Eigen::MatrixXd &cov)
{
    throw NotImplementedException("`ProcessSpaceMeanAndCovariance` needs to be implemented.");
}

void BaseUKF::MeasurementSpaceMeanAndCovariance(
        const Eigen::MatrixXd &sigma_pts,
        const SensorDataPacket::SensorType &sensor_type,
        Eigen::VectorXd &mean,
        Eigen::MatrixXd &cov)
{
    throw NotImplementedException("`MeasurementSpaceMeanAndCovariance` needs to be implemented.");
}