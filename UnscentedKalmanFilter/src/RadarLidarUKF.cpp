//
// Created by jacob on 4/18/2017.
//

#include "RadarLidarUKF.h"

using namespace Eigen;

RadarLidarUKF::RadarLidarUKF(
            std::vector<float> radar_noise,
            std::vector<float> lidar_noise,
            std::vector<float> process_noise,
            double lambda) :
        BaseUKF(5, process_noise, lambda),
        R_radar_(MatrixXd(3, 3)),
        R_lidar_(MatrixXd(2, 2))
{
    // radar noise covariance matrix
    R_radar_ << pow(radar_noise[0], 2), 0, 0,
                0, pow(radar_noise[1], 2), 0,
                0, 0, pow(radar_noise[2], 2);

    // lidar noise covariance matrix
    R_lidar_ << pow(lidar_noise[0], 2), 0,
                0, pow(lidar_noise[1], 2);
}


Eigen::MatrixXd RadarLidarUKF::PredictSigmaPoints(const Eigen::MatrixXd &sigma_pts, const double delta_t)
{
    int n_sigma_points = 2 * n_aug_states_ + 1;
    MatrixXd sig_pred = MatrixXd(n_states_, n_sigma_points);

    for (int i = 0; i < n_sigma_points; ++i)
    {
        double px = sigma_pts(0, i);
        double py = sigma_pts(1, i);
        double v  = sigma_pts(2, i);
        double yaw = sigma_pts(3, i);
        double yawd = sigma_pts(4, i);
        double nu_acc = sigma_pts(5, i);
        double nu_yawdd = sigma_pts(6, i);

        double delta_t2 = pow(delta_t, 2);

        // predicted state values
        double p_px, p_py;

        // don't divide by zero
        if (fabs(yawd) > 1e-3) {
            p_px = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
            p_py = py + v/yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
        } else {
            p_px = px + v * delta_t * cos(yaw);
            p_py = py + v * delta_t * sin(yaw);
        }

        // add noise
        p_px += 0.5 * nu_acc * delta_t2 * cos(yaw);
        p_py += 0.5 * nu_acc * delta_t2 * sin(yaw);

        double p_v = v + nu_acc*delta_t;
        double p_yaw = yaw + delta_t*yawd + 0.5*nu_yawdd*delta_t2;
        double p_yawd = yawd + nu_yawdd*delta_t;

        sig_pred(0, i) = p_px;
        sig_pred(1, i) = p_py;
        sig_pred(2, i) = p_v;
        sig_pred(3, i) = p_yaw;
        sig_pred(4, i) = p_yawd;
    }

    return sig_pred;
}

Eigen::MatrixXd RadarLidarUKF::SigmaPointsToMeasurementSpace(
        const Eigen::MatrixXd &sigma_pts,
        const Eigen::VectorXd &weights,
        const SensorDataPacket::SensorType sensor_type)
{
    MatrixXd meas_space_sigma_pts;
    int n_sigma_points = sigma_pts.cols();

    if (sensor_type == SensorDataPacket::RADAR)
    {
        meas_space_sigma_pts = MatrixXd(3, n_sigma_points);

        for (int i = 0; i < n_sigma_points; ++i)
        {
            // extract values for better readability
            double px = sigma_pts(0, i);
            double py = sigma_pts(1, i);
            double v = sigma_pts(2, i);
            double yaw = sigma_pts(3, i);

            double vx = v * cos(yaw);
            double vy = v * sin(yaw);

            double d = sqrt(px*px + py*py);

            // don't divide be too small of a value
            if (d < 1e-3)
                d = 1e-3;

            // measurement model
            meas_space_sigma_pts(0, i) = d;                       // rho
            meas_space_sigma_pts(1, i) = atan2(py, px);           // phi
            meas_space_sigma_pts(2, i) = (px * vx + py * vy) / d; // rho_dot
        }
    }  // end radar
    else if (sensor_type == SensorDataPacket::LIDAR)
    {
        meas_space_sigma_pts = sigma_pts.block(0, 0, 2, n_sigma_points);
    }  // end lidar

    return meas_space_sigma_pts;
}

void RadarLidarUKF::ProcessSpaceMeanAndCovariance(
        const Eigen::MatrixXd &sigma_pts,
        Eigen::VectorXd &mean,
        Eigen::MatrixXd &cov)
{
    mean = sigma_pts * weights_;

    cov = MatrixXd(sigma_pts.rows(), sigma_pts.rows());
    cov.fill(0.0);
    for (int i = 0; i < 2 * n_aug_states_ + 1; ++i)
    {
        VectorXd diff = sigma_pts.col(i) - mean;

        //angle normalization
        while (diff(3) >  M_PI)
            diff(3) -= 2.0*M_PI;
        while (diff(3) < -M_PI)
            diff(3) += 2.0 * M_PI;

        cov += weights_(i) * diff * diff.transpose();
    }
}

void RadarLidarUKF::MeasurementSpaceMeanAndCovariance(
        const Eigen::MatrixXd &sigma_pts,
        const SensorDataPacket::SensorType &sensor_type,
        Eigen::VectorXd &mean,
        Eigen::MatrixXd &cov)
{
    mean = sigma_pts * weights_;

    cov = MatrixXd(sigma_pts.rows(), sigma_pts.rows());
    cov.fill(0.0);
    for (int i = 0; i < 2 * n_aug_states_ + 1; ++i)
    {
        VectorXd diff = sigma_pts.col(i) - mean;
        cov += weights_(i) * diff * diff.transpose();
    }

    cov += (sensor_type == SensorDataPacket::RADAR ? R_radar_ : R_lidar_);
}
