//
// Created by jacob on 4/18/2017.
//

#ifndef UNSCENTEDKALMANFILTER_RADARLIDAREKF_H
#define UNSCENTEDKALMANFILTER_RADARLIDAREKF_H


#include "BaseEKF.h"

class RadarLidarEKF : BaseEKF
{
public:
    RadarLidarEKF(double std_rho, double std_phi, double std_drho);

private:
    Eigen::MatrixXd PredictSigmaPoints(
            const Eigen::MatrixXd &sigma_pts,
            const double delta_t);

    Eigen::MatrixXd SigmaPointsToMeasurementSpace(
            const Eigen::MatrixXd &sigma_pts,
            const Eigen::VectorXd &weights,
            const SensorDataPacket::SensorType sensor_type);

    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd R_lidar_;
};


#endif //UNSCENTEDKALMANFILTER_RADARLIDAREKF_H
