//
// Created by jaket on 4/15/2017.
//

#ifndef UNSCENTEDKALMANFILTER_TOOLS_H
#define UNSCENTEDKALMANFILTER_TOOLS_H

#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "sensor_data_packet.h"

Eigen::VectorXd polar_to_cartesian(const Eigen::VectorXd &polar);

Eigen::VectorXd calculate_rmse(std::vector<SensorDataPacket> &data_packets);

void check_arguments(int argc, char* argv[]);

void check_files(
        const std::ifstream &infile,
        const std::string &in_name,
        const std::ofstream &outfile,
        const std::string &out_name);

#endif //UNSCENTEDKALMANFILTER_TOOLS_H
