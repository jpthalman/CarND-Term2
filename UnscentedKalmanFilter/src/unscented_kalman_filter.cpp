//
// Created by jaket on 4/13/2017.
//

#include "unscented_kalman_filter.h"
#include "tools.h"

using namespace Eigen;


UnscentedKalmanFilter::UnscentedKalmanFilter(int init_n_states = 5, float std_acc = 0.2, float std_yawdd = 0.2) :
        n_states_(init_n_states),
        n_aug_states_(init_n_states + 2),
        lambda_(3 - n_aug_states_),
        x_(Eigen::VectorXd(init_n_states)),
        P_(Eigen::MatrixXd::Identity(init_n_states, init_n_states)),
        Q_(Eigen::MatrixXd::Identity(2, 2)),
        weights_(VectorXd(2 * init_n_states + 1))
{
    // set the values for the process covariance matrix
    Q_(0, 0) = pow(std_acc, 2);
    Q_(1, 1) = pow(std_yawdd, 2);

    // initialize the weights for the prediction step
    weights_.fill(0.5 / (lambda_ + n_aug_states_));
    weights_(0) = lambda_ / (lambda_ + n_aug_states_);
}

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
    GenerateSigmaPoints(x_aug, P_aug, lambda_);

    // transform the sigma points into the new space
    PredictSigmaPoints(sigma_points_, dt);

    // predict the new mean and covariance
    PredictMeanAndCovariance(sigma_points_);

    /**********************************************************************************
     *                                      Update
     **********************************************************************************/

    // TODO

    // store the UKF prediction in the referenced data packet
    data.predictions = x_;
    return;
}

void UnscentedKalmanFilter::GenerateSigmaPoints(Eigen::VectorXd &x, Eigen::MatrixXd &P, int lambda) {
    int n_states = x.size();
    double scaling_factor = sqrt(n_states + lambda);
    MatrixXd P_sqrt = P.llt().matrixL();

    sigma_points_ = MatrixXd(n_aug_states_, 2 * n_aug_states_ + 1);
    sigma_points_ << x, (scaling_factor * P_sqrt).colwise() + x, (-scaling_factor * P_sqrt).colwise() + x;
}

void UnscentedKalmanFilter::PredictSigmaPoints(Eigen::MatrixXd &sigma_pts, double delta_t) {
    MatrixXd sig_pred = MatrixXd(n_states_, 2 * n_aug_states_ + 1);

    for (int i = 0; i < 2 * n_aug_states_ + 1; ++i) {
        double px = sigma_pts(0, i);
        double py = sigma_pts(1, i);
        double v  = sigma_pts(2, i);
        double yaw = sigma_pts(3, i);
        double yawd = sigma_pts(4, i);
        double nu_acc = sigma_pts(5, i);
        double nu_yawdd = sigma_pts(6, i);

        // predicted state values
        double p_px, p_py;

        // don't divide by zero
        if (fabs(yawd) > 1e-3) {
            p_px = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
            p_py = py + v/yawd * (cos(yaw + yawd*delta_t) - cos(yaw));
        } else {
            p_px = px + v * delta_t * cos(yaw);
            p_py = py + v * delta_t * sin(yaw);
        }

        double p_yaw = yaw + delta_t * yawd;

        // add noise
        p_px += 0.5 * nu_acc * pow(delta_t, 2) * cos(yaw);
        p_py += 0.5 * nu_acc * pow(delta_t, 2) * sin(yaw);

        sig_pred(0, i) = p_px;
        sig_pred(1, i) = p_py;
        sig_pred(2, i) = v;
        sig_pred(3, i) = p_yaw;
        sig_pred(4, i) = yawd;
    }

    sigma_pts = sig_pred;
}

void UnscentedKalmanFilter::PredictMeanAndCovariance(Eigen::MatrixXd &sigma_pts) {
    x_ = sigma_pts * weights_;

    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_states_ + 1; ++i) {
        VectorXd diff = sigma_pts.col(i) - x_;

        // normalize the angles
        while (diff(3) > M_PI)
            diff(3) -= 2. * M_PI;
        while (diff(3) < M_PI)
            diff(3) += 2. * M_PI;

        P_ += weights_(i) * diff * diff.transpose();
    }
}
