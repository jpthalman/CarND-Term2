//
// Created by jacob on 4/18/2017.
//

#ifndef UNSCENTEDKALMANFILTER_RADARLIDAREKF_H
#define UNSCENTEDKALMANFILTER_RADARLIDAREKF_H


#include "BaseEKF.h"

class RadarLidarEKF : BaseEKF
{
public:
    RadarLidarEKF() : BaseEKF(5, std::vector<double>{0.2, 0.2}, -5) {}

private:
    void PredictSigmaPoints(Eigen::MatrixXd &sigma_pts, const double delta_t);
    void SigmaPointsToMeasurementSpace(Eigen::MatrixXd &sigma_pts,
                                                  const SensorDataPacket::SensorType sensor_type);
};


#endif //UNSCENTEDKALMANFILTER_RADARLIDAREKF_H
