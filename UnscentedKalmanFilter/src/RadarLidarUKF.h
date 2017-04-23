//
// Created by jacob on 4/18/2017.
//

#ifndef UNSCENTEDKALMANFILTER_RADARLIDARUKF_H
#define UNSCENTEDKALMANFILTER_RADARLIDARUKF_H


#include "BaseUKF.h"

class RadarLidarUKF : BaseUKF
{
public:
    RadarLidarUKF(
            std::vector<float> radar_noise,
            std::vector<float> lidar_noise,
            std::vector<float> process_noise,
            double lambda);

    ~RadarLidarUKF() {};

    using BaseUKF::GetCurrentState;
    using BaseUKF::GetCurrentCovariance;
    using BaseUKF::ProcessMeasurement;

private:
    Eigen::VectorXd InitializeState(
            const SensorDataPacket &data);

    Eigen::MatrixXd PredictSigmaPoints(
            const Eigen::MatrixXd &sigma_pts,
            const double delta_t);

    Eigen::MatrixXd SigmaPointsToMeasurementSpace(
            const Eigen::MatrixXd &sigma_pts,
            const Eigen::VectorXd &weights,
            const SensorDataPacket::SensorType sensor_type);

    void ProcessSpaceMeanAndCovariance(
            const Eigen::MatrixXd &sigma_pts,
            Eigen::VectorXd &mean,
            Eigen::MatrixXd &cov);

    void MeasurementSpaceMeanAndCovariance(
            const Eigen::MatrixXd &sigma_pts,
            const SensorDataPacket::SensorType &sensor_type,
            Eigen::VectorXd &mean,
            Eigen::MatrixXd &cov);

    Eigen::VectorXd StateSpaceToCartesian(const Eigen::VectorXd &x);

    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd R_lidar_;
};


#endif //UNSCENTEDKALMANFILTER_RADARLIDARUKF_H
