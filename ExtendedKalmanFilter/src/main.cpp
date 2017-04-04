#include <iostream>
#include <vector>
#include "FusionEKF.h"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]) {
    // ensure that an infile and an outfile are provided
    check_arguments(argc, argv);

    string infile_name = argv[1];
    ifstream infile(infile_name.c_str(), ifstream::in);

    string outfile_name = argv[2];
    ofstream outfile(outfile_name.c_str(), ofstream::out);

    // ensure that the infile/outfile were opened correctly
    check_files(infile, infile_name, outfile, outfile_name);

    // create lists to store the measurement and corresponding ground truth
    vector<SensorDataPacket> measurement_packet_list;
    vector<SensorDataPacket> ground_truth_packet_list;

    string line;
    while (getline(infile, line)) {

        istringstream iss(line);
        string sensor_type;
        SensorDataPacket data_packet;

        // read in sensor data
        iss >> sensor_type;
        if (sensor_type == "L") {
            data_packet.sensor_type = SensorDataPacket::LIDAR;
            data_packet.values = VectorXd(2);

            float x, y;
            iss >> x >> y;

            data_packet.values << x, y;
            iss >> data_packet.timestamp;
            measurement_packet_list.push_back(data_packet);
        }
        else if (sensor_type == "R") {
            data_packet.sensor_type = SensorDataPacket::RADAR;
            data_packet.values = VectorXd(3);

            float rho, phi, rho_dot;
            iss >> rho >> phi >> rho_dot;

            data_packet.values << rho, phi, rho_dot;
            iss >> data_packet.timestamp;
            measurement_packet_list.push_back(data_packet);
        }

        // read in ground truth data
        float px, py, vx, vy;
        iss >> px >> py >> vx >> vy;

        data_packet.values = VectorXd(4);
        data_packet.values << px, py, vx, vy;
        ground_truth_packet_list.push_back(data_packet);
    }

    // create extended kalman filter instance
    FusionEKF ekf;

    // used to compute RMSE later
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truths;

    VectorXd state;

    // output the measurements
    for (size_t obs = 0; obs < measurement_packet_list.size(); ++obs) {
        state = ekf.ProcessMeasurement(measurement_packet_list[obs]);

        for (size_t k = 0; k < state.size(); ++k)
            outfile << state[k] << "\t";

        if (measurement_packet_list[obs].sensor_type == SensorDataPacket::RADAR) {
            outfile << measurement_packet_list[obs].values(0) << "\t"; // px
            outfile << measurement_packet_list[obs].values(1) << "\t"; // py
        }
        else if (measurement_packet_list[obs].sensor_type == SensorDataPacket::LIDAR) {
            // transform the measurements from polar to cartesian and output
            double rho = measurement_packet_list[obs].values(0);
            double phi = measurement_packet_list[obs].values(1);

            outfile << rho * sin(phi) << "\t"; // px
            outfile << rho * cos(phi) << "\t"; // py
        }

        // output the ground truth values
        for (int i = 0; i < 4; ++i)
            outfile << ground_truth_packet_list[obs].values(i) << "\t";
        outfile << endl;

        estimations.push_back(ekf.GetCurrentState());
        ground_truths.push_back(ground_truth_packet_list[obs].values);
    }

    cout << "Acuracy - RMSE:" << endl
         << calculate_rmse(estimations, ground_truths) << endl;

    // close files
    if (outfile.is_open())
        outfile.close();
    if (infile.is_open())
        infile.close();

    return 0;
}