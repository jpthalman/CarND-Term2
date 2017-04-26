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
 *  - NormalizeMeasurementVector
 *  - NormalizeStateVector
 *
 * @param n_states: The number of states that the UKF will track.
 * @param noise_stdevs: Vector of the noise values that correspond to the tracked states.
 * @param lambda: Scaling parameter for the sigma points.
 * */
BaseUKF::BaseUKF(int n_states,
                 std::vector<float> noise_stdevs,
                 double lambda) :
        n_states_(n_states),
        n_noise_coeffs_(noise_stdevs.size()),
        n_aug_states_(n_states + n_noise_coeffs_),
        lambda_(lambda),
        x_(VectorXd(n_states)),
        P_(MatrixXd::Identity(n_states, n_states)),
        P_aug_(MatrixXd(n_aug_states_, n_aug_states_)),
        X_sigma_points_(MatrixXd(n_aug_states_, 2 * n_aug_states_ + 1)),
        weights_(VectorXd(2 * n_aug_states_ + 1)),
        is_initialized_(false)
{
    // instantiate the augmented covariance matrix with the process noise
    P_aug_.fill(0.0);
    for (int i = n_states_; i < n_aug_states_; ++i)
        P_aug_(i, i) = pow(noise_stdevs[i - n_states_], 2);

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
        x_ = InitializeState(data);
        return;
    }

    /**********************************************************************************
     *                                     Predict
     **********************************************************************************/

    // calculate the time step in seconds since the last prediction
    double dt = (data.timestamp - prev_timestamp_) / 1e6;
    prev_timestamp_ = data.timestamp;

    PredictState(data, dt);

    /**********************************************************************************
     *                                      Update
     **********************************************************************************/

    VectorXd resid = UpdateState(data.observations, data.sensor_type);

    // store the NIS and predictions in the data packet
    data.nis = resid.transpose() * S_.inverse() * resid;
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
        const VectorXd &x,
        const MatrixXd &P)
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
void BaseUKF::CalculateKalmanGain(const SensorDataPacket::SensorType sensor_type)
{
    MatrixXd cross_correlation = MatrixXd(x_.size(), z_.size());
    cross_correlation.fill(0.0);

    for (int i = 0; i < weights_.size(); ++i) {
        VectorXd x_diff = X_sigma_points_.col(i) - x_;
        x_diff = NormalizeStateVector(x_diff);

        VectorXd z_diff = Z_sigma_points_.col(i) - z_;
        z_diff = NormalizeMeasurementVector(z_diff, sensor_type);

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
void BaseUKF::PredictState(const SensorDataPacket &data, const double dt)
{
    // create augmented state and covariance to include noise
    VectorXd x_aug = VectorXd(n_aug_states_);
    x_aug << x_, 0, 0;
    P_aug_.topLeftCorner(n_states_, n_states_) = P_;

    // create the initial sigma points
    GenerateSigmaPoints(x_aug, P_aug_);

    // predict the sigma points to t+1
    X_sigma_points_ = PredictSigmaPoints(X_sigma_points_, dt);

    // predict the new mean and covariance with the new sigma points
    Gaussian predictions = ProcessSpaceMeanAndCovariance(X_sigma_points_);
    x_ = predictions.first;
    P_ = predictions.second;
}

VectorXd BaseUKF::UpdateState(const VectorXd &measurement, const SensorDataPacket::SensorType &sensor_type)
{
    // transform sigma points into the measurement space
    Z_sigma_points_ = SigmaPointsToMeasurementSpace(X_sigma_points_, weights_, sensor_type);

    // get the mean and covariance of the new sigma points
    Gaussian estimates =  MeasurementSpaceMeanAndCovariance(Z_sigma_points_, sensor_type);
    z_ = estimates.first;
    S_ = estimates.second;

    // calculate the kalman gain
    CalculateKalmanGain(sensor_type);

    // calculate and normalize the residual
    VectorXd z_diff = NormalizeMeasurementVector(measurement - z_, sensor_type);

    // update the state and covariance
    x_ += K_ * z_diff;
    P_ -= K_ * S_ * K_.transpose();

    return z_diff;
}
