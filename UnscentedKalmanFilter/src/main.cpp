#include <iostream>
#include <fstream>
#include "sensor_data_packet.h"
#include "RadarLidarUKF.h"
#include "tools.h"

using namespace std;

int main(int argc, char* argv[])
{
    check_arguments(argc, argv);

    string in_name = argv[1], out_name = argv[2];
    ifstream infile(in_name.c_str(), ifstream::in);
    ofstream outfile(out_name.c_str(), ofstream::out);

    check_files(infile, in_name, outfile, out_name);

    vector<SensorDataPacket> data_packets;

    string line;
    while (getline(infile, line))
    {
        istringstream iss(line);
        SensorDataPacket sensor_reading(iss);
        data_packets.push_back(sensor_reading);
    }

    vector<float> radar_noise{
            0.3,   // rho
            0.03,  // phi
            0.3    // rho_dot
    };
    vector<float> lidar_noise{
            0.15,  // px
            0.15   // py
    };
    vector<float> process_noise{
            0.2,  // acceleration
            0.2   // yaw rate
    };
    double lambda = -5;

    RadarLidarUKF ukf(radar_noise, lidar_noise, process_noise, lambda);

    for (auto &data_packet : data_packets)
    {
        ukf.ProcessMeasurement(data_packet);

        for (size_t i = 0; i < data_packet.predictions.size(); ++i)
            outfile << data_packet.predictions(i) << "\t";

        if (data_packet.sensor_type == SensorDataPacket::LIDAR)
        {
            outfile << data_packet.observations(0) << "\t"  // px
                    << data_packet.observations(1) << "\t"; // py
        }
        else if (data_packet.sensor_type == SensorDataPacket::RADAR)
        {
            Eigen::VectorXd cartesian = polar_to_cartesian(data_packet.observations);

            outfile << cartesian(0) << "\t"  // px
                    << cartesian(1) << "\t"; // py
        }

        for (size_t i = 0; i < data_packet.ground_truths.size(); ++i)
            outfile << data_packet.ground_truths(i) << "\t";
        outfile << endl;
    }

    cout << "Acuracy - RMSE:" << endl
         << calculate_rmse(data_packets) << endl;

    if (infile.is_open())
        infile.close();
    if (outfile.is_open())
        outfile.close();

    return 0;
}