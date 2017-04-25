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
        try {
            x_ = InitializeState(data);
        }
        catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }

        return;
    }

    /**********************************************************************************
     *                                     Predict
     **********************************************************************************/

    // calculate the time step in seconds since the last prediction
    double dt = (data.timestamp - prev_timestamp_) / 1e6;
    prev_timestamp_ = data.timestamp;

    // if the timestep is too large, break up the prediction into multiple steps
    const double dt_max = 0.1;
    while (dt > dt_max)
    {
        Predict(data, dt_max);
        dt -= dt_max;
    }
    Predict(data, dt);

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
    VectorXd z_diff = data.observations - z_;

    if (data.sensor_type == SensorDataPacket::RADAR)
    {
        while (z_diff(1) > M_PI)
            z_diff(1) -= 2.0 * M_PI;
        while (z_diff(1) < -M_PI)
            z_diff(1) += 2.0 * M_PI;
    }

    std::cout << "Update" << std::endl << std::endl;

    x_ += K_ * z_diff;
    P_ -= K_ * S_ * K_.transpose();

    data.nis = z_diff.transpose() * S_.inverse() * z_diff;
    data.predictions = StateSpaceToCartesian(x_);

    std::cout << "----------- "
              << (data.sensor_type == SensorDataPacket::RADAR ? "RADAR" : "LIDAR")
              << " -----------" << std::endl << std::endl
              << "State:" << std::endl
              << x_ << std::endl << std::endl
              << "Covariance:" << std::endl
              << P_ << std::endl << std::endl;

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

        while (x_diff(3) > M_PI)
            x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI)
            x_diff(3) += 2.0 * M_PI;

        VectorXd z_diff = Z_sigma_points_.col(i) - z_;

        if (z_.size() == 3)
        {
            while (z_diff(1) > M_PI)
                z_diff(1) -= 2.0 * M_PI;
            while (z_diff(1) < -M_PI)
                z_diff(1) += 2.0 * M_PI;
        }

        cross_correlation += weights_(i) * x_diff * z_diff.transpose();
    }

    K_ = cross_correlation * S_.inverse();
}

void BaseUKF::Predict(const SensorDataPacket &data, const double dt)
{
    // create augmented state and covariance to include noise
    VectorXd x_aug = VectorXd(n_aug_states_);
    x_aug << x_, 0, 0;

    int n_noise_coeff = n_aug_states_ - n_states_;
    MatrixXd P_aug = MatrixXd(n_aug_states_, n_aug_states_);
    P_aug.fill(0.0);

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

VectorXd BaseUKF::StateSpaceToCartesian(
        const VectorXd &x)
{
    throw NotImplementedException("`StateSpaceToCartesian` needs to be implemented.");
}

Eigen::VectorXd BaseUKF::InitializeState(
        const SensorDataPacket &data)
{
    throw NotImplementedException("`InitializeState` needs to be implemented.");
}
