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
    ifstream infile(in_name.c_str());
    ofstream outfile(out_name.c_str());

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
            3.0,  // acceleration
            2.0   // yaw rate
    };
    double lambda = -4;

    RadarLidarUKF ukf(radar_noise, lidar_noise, process_noise, lambda);

    outfile << "SensorType,Px_pred,Py_pred,Vx_pred,Vy_pred,Px_meas,Py_meas,Px_gt,Py_gt,Vx_gt,Vy_gt,NIS" << endl;

    for (auto &data_packet : data_packets)
    {
        ostringstream line_out;

        ukf.ProcessMeasurement(data_packet);

        line_out << (data_packet.sensor_type == SensorDataPacket::RADAR ? "R" : "L") << ",";

        for (size_t i = 0; i < data_packet.predictions.size(); ++i)
            line_out << data_packet.predictions(i) << ",";

        if (data_packet.sensor_type == SensorDataPacket::LIDAR)
        {
            line_out << data_packet.observations(0) << ","  // px
                     << data_packet.observations(1) << ","; // py
        }
        else if (data_packet.sensor_type == SensorDataPacket::RADAR)
        {
            Eigen::VectorXd cartesian = polar_to_cartesian(data_packet.observations);

            line_out << cartesian(0) << ","  // px
                     << cartesian(1) << ","; // py
        }

        for (size_t i = 0; i < data_packet.ground_truths.size(); ++i)
            line_out << data_packet.ground_truths(i) << ",";

        line_out << data_packet.net_innovation_score;
        outfile << line_out.str() << endl;
    }

    cout << "Acuracy - RMSE:" << endl
         << calculate_rmse(data_packets) << endl;

    if (infile.is_open())
        infile.close();
    if (outfile.is_open())
        outfile.close();

    return 0;
}