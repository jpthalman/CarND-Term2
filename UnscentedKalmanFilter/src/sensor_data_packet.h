//
// Created by japata on 4/10/17.
//

#ifndef UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H
#define UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H

#include <iostream>
#include "Eigen/Dense"

struct SensorDataPacket {
    long long timestamp;

    enum SensorType {
        RADAR,
        LIDAR
    } sensor_type;

    Eigen::VectorXd observations;
    Eigen::VectorXd ground_truths;
};

/**
 * Read data from a line in a text file (istringstream) into a SensorDataPacket.
 * */
void store_sensor_data(std::istringstream &iss, SensorDataPacket &data);

#endif //UNSCENTEDKALMANFILTER_SENSOR_DATA_PACKET_H
