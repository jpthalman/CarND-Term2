//
// Created by jacob on 4/18/2017.
//

#include "RadarLidarUKF.h"
#include "tools.h"

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

VectorXd RadarLidarUKF::InitializeState(
        const SensorDataPacket &data)
{
    VectorXd x = VectorXd(5);
    x.fill(0.0);

    if (data.sensor_type == SensorDataPacket::LIDAR)
    {
        x.fill(0.0);
        x(0) = data.observations(0);  // px
        x(1) = data.observations(1);  // py
        x(3) = atan2(x(1), x(0));     // yaw
    }
    else if (data.sensor_type == SensorDataPacket::RADAR)
    {
        double rho = data.observations(0);
        double phi = data.observations(1);
        double drho = data.observations(2);

        x.fill(0.0);
        x(0) = rho * cos(phi);  // px
        x(1) = rho * sin(phi);  // py
        x(2) = drho;            // v
        x(3) = phi;             // yaw
    }

    prev_timestamp_ = data.timestamp;

    // only initialize if there is an actual measurement
    if (fabs(x(0)) + fabs(x(1)) > 1e-3)
        is_initialized_ = true;
    return x;
}

Eigen::MatrixXd RadarLidarUKF::PredictSigmaPoints(
        const Eigen::MatrixXd &sigma_pts,
        const double delta_t)
{
    int n_sigma_points = 2 * n_aug_states_ + 1;
    MatrixXd sig_pred = MatrixXd(n_states_, n_sigma_points);

    // for each sigma point
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
            p_px = px + v*cos(yaw)*delta_t;
            p_py = py + v*sin(yaw)*delta_t;
        }

        // add noise
        p_px += 0.5*delta_t2*cos(yaw)*nu_acc;
        p_py += 0.5*delta_t2*sin(yaw)*nu_acc;

        // predicted velocity, yaw, and yaw rate
        double p_v = v + nu_acc*delta_t;
        double p_yaw = yaw + delta_t*yawd + 0.5*delta_t2*nu_yawdd;
        double p_yawd = yawd + nu_yawdd*delta_t;

        // store the predictions in their sigma point column
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

        // for each sigma point
        for (int i = 0; i < n_sigma_points; ++i)
        {
            // extract values for better readability
            double px = sigma_pts(0, i);
            double py = sigma_pts(1, i);
            double v = sigma_pts(2, i);
            double yaw = sigma_pts(3, i);

            // velocities
            double vx = v * cos(yaw);
            double vy = v * sin(yaw);

            // don't divide by too small of a value
            if (fabs(px) < 1e-3)
                px = 1e-3;
            if (fabs(py) < 1e-3)
                py = 1e-3;

            double d = sqrt(px*px + py*py);

            // measurement model
            meas_space_sigma_pts(0, i) = d;                     // rho
            meas_space_sigma_pts(1, i) = atan2(py, px);         // phi
            meas_space_sigma_pts(2, i) = (px*vx + py*vy) / d;   // rho_dot
        }
    }  // end radar
    else if (sensor_type == SensorDataPacket::LIDAR)
    {
        meas_space_sigma_pts = sigma_pts.block(0, 0, 2, n_sigma_points);
    }  // end lidar

    return meas_space_sigma_pts;
}

Gaussian RadarLidarUKF::ProcessSpaceMeanAndCovariance(
        const Eigen::MatrixXd &sigma_pts)
{
    VectorXd mean = sigma_pts * weights_;

    MatrixXd cov = MatrixXd(sigma_pts.rows(), sigma_pts.rows());
    cov.fill(0.0);

    // for each sigma point
    for (int i = 0; i < 2 * n_aug_states_ + 1; ++i)
    {
        VectorXd diff = sigma_pts.col(i) - mean;
        diff = NormalizeStateVector(diff);
        cov += weights_(i) * diff * diff.transpose();
    }

    return std::make_pair(mean, cov);
}

Gaussian RadarLidarUKF::MeasurementSpaceMeanAndCovariance(
        const Eigen::MatrixXd &sigma_pts,
        const SensorDataPacket::SensorType &sensor_type)
{
    VectorXd mean = sigma_pts * weights_;

    MatrixXd cov = MatrixXd(sigma_pts.rows(), sigma_pts.rows());
    cov.fill(0.0);

    // for each sigma point
    for (int i = 0; i < 2 * n_aug_states_ + 1; ++i)
    {
        VectorXd diff = sigma_pts.col(i) - mean;

        //angle normalization
        diff = NormalizeMeasurementVector(diff, sensor_type);

        cov += weights_(i) * diff * diff.transpose();
    }

    cov += (sensor_type == SensorDataPacket::RADAR ? R_radar_ : R_lidar_);
    return std::make_pair(mean, cov);
}

Eigen::VectorXd RadarLidarUKF::StateSpaceToCartesian(const Eigen::VectorXd &x)
{
    Vector4d cartesian;
    cartesian <<    x(0), // px
                    x(1), // py
                    x(2)*cos(x(3)), // vx
                    x(2)*sin(x(3)); // vy
    return cartesian;
}

VectorXd RadarLidarUKF::NormalizeStateVector(
        const VectorXd &x)
{
    VectorXd x_norm = x;

    // normalize the angle
    while (x_norm(3) > M_PI)
        x_norm(3) -= 2.0 * M_PI;
    while (x_norm(3) < -M_PI)
        x_norm(3) += 2.0 * M_PI;

    return x_norm;
}

VectorXd RadarLidarUKF::NormalizeMeasurementVector(
        const VectorXd &z,
        SensorDataPacket::SensorType sensor_type)
{
    VectorXd z_norm = z;

    if (sensor_type == SensorDataPacket::RADAR)
    {
        // normalize the angle
        while (z_norm(1) > M_PI)
            z_norm(1) -= 2.0 * M_PI;
        while (z_norm(1) < -M_PI)
            z_norm(1) += 2.0 * M_PI;
    }

    return z_norm;
}
