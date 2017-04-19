//
// Created by jacob on 4/18/2017.
//

#include "RadarLidarEKF.h"

using namespace Eigen;

RadarLidarEKF::RadarLidarEKF(double std_rho, double std_phi, double std_drho) :
        BaseEKF(5, std::vector<double>{0.2, 0.2}, -5),
        R_radar_(MatrixXd(3, 3))
{
    // radar noise covariance matrix
    R_radar_ << pow(std_rho, 2), 0, 0,
                0, pow(std_phi, 2), 0,
                0, 0, pow(std_drho, 2);
}


Eigen::MatrixXd RadarLidarEKF::PredictSigmaPoints(const Eigen::MatrixXd &sigma_pts, const double delta_t)
{
    int n_sigma_points = 2 * n_aug_states_ + 1;
    MatrixXd sig_pred = MatrixXd(n_states_, n_sigma_points);

    for (int i = 0; i < n_sigma_points; ++i) {
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

    return sig_pred;
}

Eigen::MatrixXd RadarLidarEKF::SigmaPointsToMeasurementSpace(
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

            // measurement model
            meas_space_sigma_pts(0, i) = sqrt(px*px + py*py);                              //r
            meas_space_sigma_pts(1, i) = atan2(py, px);                                    //phi
            meas_space_sigma_pts(2, i) = (px * vx + py * vy) / meas_space_sigma_pts(0, i); //r_dot
        }

        return meas_space_sigma_pts;
    }  // end radar
    else if (sensor_type == SensorDataPacket::LIDAR)
    {
        meas_space_sigma_pts = sigma_pts.block(0, 0, 2, n_sigma_points);
        return meas_space_sigma_pts;
    }  // end lidar
}
