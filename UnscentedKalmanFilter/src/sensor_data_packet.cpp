//
// Created by jaket on 4/13/2017.
//

#include "sensor_data_packet.h"

SensorDataPacket::SensorDataPacket(
        long long init_time,
        char init_sensor,
        Eigen::VectorXd init_obs,
        Eigen::VectorXd init_gt)
{
    timestamp = init_time;

    assert(init_sensor == 'L' || init_sensor == 'R');
    sensor_type = init_sensor == 'R' ? SensorType::RADAR : SensorType::LIDAR;

    observations = init_obs;
    ground_truths = init_gt;
    predictions = Eigen::VectorXd::Zero(4);
}

SensorDataPacket::SensorDataPacket(std::istringstream &iss)
{
    char sensor_type_char;
    iss >> sensor_type_char;

    if (sensor_type_char == 'L') {
        sensor_type = SensorDataPacket::LIDAR;

        double px, py;
        iss >> px >> py;

        observations = Eigen::VectorXd(2);
        observations << px, py;
    }
    else if (sensor_type_char == 'R') {
        sensor_type = SensorDataPacket::RADAR;

        double rho, phi, drho;
        iss >> rho >> phi >> drho;

        observations = Eigen::VectorXd(3);
        observations << rho, phi, drho;
    }

    long long obs_timestamp;
    double px, py, vx, vy;

    iss >> obs_timestamp >> px >> py >> vx >> vy;
    timestamp = obs_timestamp;

    ground_truths = Eigen::VectorXd(4);
    ground_truths << px, py, vx, vy;

    predictions = Eigen::VectorXd::Zero(4);
}
