//
// Created by jaket on 4/15/2017.
//

#include "tools.h"
#include <fstream>

using namespace std;
using namespace Eigen;

Eigen::VectorXd polar_to_cartesian(const Eigen::VectorXd &polar) {
    double rho = polar(0);
    double phi = polar(1);
    double rho_dot = polar(2);

    Eigen::VectorXd cartesian = Eigen::Vector4d::Zero();
    cartesian <<    rho * cos(phi), // px
            rho * sin(phi),         // py
            rho_dot * cos(phi),     // vx
            rho_dot * sin(phi);     // vy
    return cartesian;
}

Eigen::VectorXd calculate_rmse(std::vector<SensorDataPacket> &data_packets)
{
    size_t n_obs = data_packets.size();
    VectorXd rmse = Vector4d::Zero();

    if (n_obs == 0)
    {
        cerr << "calculate_rmse | There are no data packets." << endl;
        return rmse;
    }

    Vector4d resid;
    for (size_t i = 0; i < n_obs; ++i)
    {
        resid  = data_packets[i].ground_truths - data_packets[i].predictions;
        resid = resid.array() * resid.array();
        rmse += resid;
    }

    rmse /= n_obs;

    return rmse.array().sqrt();
}

void check_arguments(int argc, char* argv[])
{
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/infile.txt path/to/outfile.txt";

    bool has_valid_flags = false;

    if (argc == 1)
        cerr << usage_instructions << endl;
    else if (argc == 2)
        cerr << "Please include and output file.\n" << usage_instructions << endl;
    else if (argc == 3)
        has_valid_flags = true;
    else
        cerr << "Too many arguments.\n" << usage_instructions << endl;

    if (!has_valid_flags)
        exit(EXIT_FAILURE);
}

void check_files(
        const ifstream &infile,
        const string &in_name,
        const ofstream &outfile,
        const string &out_name)
{
    if (!infile.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!outfile.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}