#ifndef EXTENDEDKALMANFILTER_SENSOR_DATA_PACKET_H
#define EXTENDEDKALMANFILTER_SENSOR_DATA_PACKET_H

#include "Eigen/Dense"

struct SensorDataPacket {
    long long timestamp_;

    enum SensorType {
        RADAR,
        LIDAR
    } sensor_type_;

    Eigen::VectorXd values_;
};

#endif //EXTENDEDKALMANFILTER_SENSOR_DATA_PACKET_H
