#include <iostream>
#include <fstream>
#include "sensor_data_packet.h"
#include "RadarLidarUKF.h"
#include "tools.h"

using namespace std;

int main(int argc, char* argv[])
{
    // make sure there is an infile and outfile provided
    check_arguments(argc, argv);

    string in_name = argv[1], out_name = argv[2];
    ifstream infile(in_name.c_str());
    ofstream outfile(out_name.c_str());

    // make sure the provided files are valid
    check_files(infile, in_name, outfile, out_name);

    vector<SensorDataPacket> data_packets;

    string line;
    while (getline(infile, line))
    {
        istringstream iss(line);
        // read in the data from the line using the istringstream initializer for SensorDataPackets
        SensorDataPacket sensor_reading(iss);
        data_packets.push_back(sensor_reading);
    }

    // set the noise vectors and spreading parameter for the UKF
    vector<float> radar_noise{
            0.3,   // rho
            0.03,  // phi
            0.6    // rho_dot
    };
    vector<float> lidar_noise{
            0.15,  // px
            0.15   // py
    };
    vector<float> process_noise{
            1.0,  // acceleration
            0.8   // yaw rate
    };
    double lambda = 0.0;

    // initialize the UKF with the noise vectors and spreading parameter
    RadarLidarUKF ukf(radar_noise, lidar_noise, process_noise, lambda);

    // CSV
    string delimiter = ",";

    // header for the output file
    outfile << "SensorType" << delimiter
            << "Px_pred" << delimiter
            << "Py_pred" << delimiter
            << "Vx_pred" << delimiter
            << "Vy_pred" << delimiter
            << "Px_meas" << delimiter
            << "Py_meas" << delimiter
            << "Px_gt" << delimiter
            << "Py_gt" << delimiter
            << "Vx_gt" << delimiter
            << "Vy_gt" << delimiter
            << "NIS" << endl;

    // write the data to the outfile
    for (auto &data_packet : data_packets)
    {
        // buffer to store the output line
        ostringstream line_out;

        // get the UKF prediction and store it in the data_packet
        ukf.ProcessMeasurement(data_packet);

        // write the sensor type to the out buffer
        line_out << (data_packet.sensor_type == SensorDataPacket::RADAR ? "R" : "L") << delimiter;

        // write the predictions to the out buffer
        for (size_t i = 0; i < data_packet.predictions.size(); ++i)
            line_out << data_packet.predictions(i) << delimiter;

        // write the observation to the out buffer
        if (data_packet.sensor_type == SensorDataPacket::LIDAR)
        {
            line_out << data_packet.observations(0) << delimiter  // px
                     << data_packet.observations(1) << delimiter; // py
        }
        else if (data_packet.sensor_type == SensorDataPacket::RADAR)
        {
            Eigen::VectorXd cartesian = polar_to_cartesian(data_packet.observations);

            line_out << cartesian(0) << delimiter  // px
                     << cartesian(1) << delimiter; // py
        }

        // write the ground truth to the out buffer
        for (size_t i = 0; i < data_packet.ground_truths.size(); ++i)
            line_out << data_packet.ground_truths(i) << delimiter;

        // write the NIS to the out buffer
        line_out << data_packet.nis;

        // write the out buffer to the outfile
        outfile << line_out.str() << endl;
    }

    cout << "Acuracy - RMSE:" << endl
         << calculate_rmse(data_packets) << endl;

    // close the files
    if (infile.is_open())
        infile.close();
    if (outfile.is_open())
        outfile.close();

    return 0;
}