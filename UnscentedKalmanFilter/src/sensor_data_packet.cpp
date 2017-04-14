//
// Created by jaket on 4/13/2017.
//

#include "sensor_data_packet.h"

void store_sensor_data(std::istringstream &iss, SensorDataPacket &data) {
    char sensor_type;
    iss >> sensor_type;

    if (sensor_type == 'L') {
        data.sensor_type = SensorDataPacket::LIDAR;

        double px, py;
        iss >> px >> py;

        data.observations = Eigen::VectorXd(2);
        data.observations << px, py;
    }
    else if (sensor_type == 'R') {
        data.sensor_type = SensorDataPacket::RADAR;

        double rho, phi, drho;
        iss >> rho >> phi >> drho;

        data.observations = Eigen::VectorXd(3);
        data.observations << rho, phi, drho;
    }

    long long obs_timestamp;
    double px, py, vx, vy;

    iss >> obs_timestamp >> px >> py >> vx >> vy;
    data.timestamp = obs_timestamp;
    data.ground_truths << px, py, vx, vy;
}