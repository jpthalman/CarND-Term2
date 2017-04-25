//
// Created by japata on 4/17/17.
//

#include "BaseUKF.h"
#include "tools.h"

using namespace Eigen;

/**
 * This class is a base class for an Unscented Kalman Filter. Meant to be inherited.
 *
 * Functions to implement:
 *  - InitializeState
 *  - PredictSigmaPoints
 *  - SigmaPointsToMeasurementSpace
 *  - ProcessSpaceMeanAndCovariance
 *  - MeasurementSpaceMeanAndCovariance
 *  - StateSpaceToCartesian
 *
 * @param n_states: The number of states that the UKF will track.
 * @param noise_stdevs: Vector of the noise values that correspond to the tracked states.
 * @param lambda: Scaling parameter for the sigma points.
 * */
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

/**
 * Takes in a SensorDataPacket and performs all necessary actions to predict the state and update
 * based upon the measurement.
 *
 * @param data: Fully initialized SensorDataPacket.
 * */
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

    // normalize the angles
    if (data.sensor_type == SensorDataPacket::RADAR)
    {
        while (z_diff(1) > M_PI)
            z_diff(1) -= 2.0 * M_PI;
        while (z_diff(1) < -M_PI)
            z_diff(1) += 2.0 * M_PI;
    }

    // update the state and covariance
    x_ += K_ * z_diff;
    P_ -= K_ * S_ * K_.transpose();

    // store the NIS and predictions in the data packet
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

/**
 * Use the provided state and covariance matrix to generate sigma points.
 *
 * @param x: State vector.
 * @param P: Covariance matrix.
 * */
void BaseUKF::GenerateSigmaPoints(
        const Eigen::VectorXd &x,
        const Eigen::MatrixXd &P)
{
    double scaling_factor = sqrt(n_aug_states_ + lambda_);
    MatrixXd P_sqrt = P.llt().matrixL();

    X_sigma_points_ = MatrixXd(n_aug_states_, 2 * n_aug_states_ + 1);
    X_sigma_points_ << x, (scaling_factor * P_sqrt).colwise() + x, (-scaling_factor * P_sqrt).colwise() + x;
}

/**
 * Using the state and measurement space sigma points, mean, and covariance matrix, calculate the
 * Kalman gain matrix and store it in K_.
 * */
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

/**
 * Predict the current state and covariance forward in time dt seconds.
 *
 * @param data: A fully initialized SensorDataPacket.
 * @param dt: The time in seconds to forecast the state forward.
 * */
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
