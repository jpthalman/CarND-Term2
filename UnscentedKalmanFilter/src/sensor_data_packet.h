//
// Created by japata on 4/10/17.
//

#ifndef UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H
#define UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H

#include "Eigen/Dense"

struct SensorDataPacket {
    long long timestamp;

    enum SensorType {
        RADAR,
        LIDAR
    } sensor_type;

    Eigen::VectorXd values;
};

#endif //UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H
