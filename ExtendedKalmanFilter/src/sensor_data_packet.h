//
// Created by japata on 3/27/17.
//

#ifndef EXTENDEDKALMANFILTER_SENSOR_DATA_PACKET_H
#define EXTENDEDKALMANFILTER_SENSOR_DATA_PACKET_H

#include "Eigen/Dense"


struct SensorDataPacket {
    long long timestamp;

    enum SensorType {
        RADAR,
        LIDAR
    } sensor_type;

    Eigen::VectorXd values;
};

#endif //EXTENDEDKALMANFILTER_SENSOR_DATA_PACKET_H
