//
// Created by japata on 4/10/17.
//

#ifndef UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H
#define UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H

#include <iostream>
#include "Eigen/Dense"

struct SensorDataPacket {
    long long timestamp;
    double nis;

    enum SensorType {
        RADAR,
        LIDAR
    } sensor_type;

    Eigen::VectorXd observations;
    Eigen::VectorXd predictions;
    Eigen::VectorXd ground_truths;

    /**
     * Constructors
     * */
    SensorDataPacket() = default;
    SensorDataPacket(long long init_time, char init_sensor, Eigen::VectorXd init_obs, Eigen::VectorXd init_gt);
    explicit SensorDataPacket(std::istringstream &iss);
};

#endif //UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H
