//
// Created by jacob on 3/28/2017.
//

#include <iostream>
#include <cstdlib>
#include "tools.h"

using namespace std;
using namespace Eigen;

VectorXd CalculateRMSE(kVectorList &estimations, kVectorList &ground_truths) {
    size_t n_obs = estimations.size();
    Vector4d rmse = Vector4d::Zero();

    if (n_obs == 0) {
        cerr << "Tools::CalculateRMSE | Number of estimations is zero." << endl;
        return rmse;
    }
    if (n_obs != ground_truths.size()) {
        cerr << "Tools::CalculateRMSE | Number of Estimations != Number of Ground Truths." << endl;
        return rmse;
    }

    // Calculate the squared residuals and accumulate in rmse
    Vector4d resid;
    for (size_t obs = 0; obs < n_obs; ++obs) {
        resid = ground_truths[obs] - estimations[obs];
        resid = resid.array() * resid.array();
        rmse += resid;
    }

    // Calculate the mean from squared residuals
    rmse /= n_obs;

    // Square root and return the result
    return rmse.array().sqrt();
}

MatrixXd calculate_jacobian(const VectorXd &z) {
    MatrixXd Hj(3, 4);

    double px = z(0);
    double py = z(1);
    double vx = z(2);
    double vy = z(3);

    // Don't divide by zero.
    if (abs(px + py) < 1e-5) {
        cerr << "Tools::CalculateJacobian | Divide by zero error." << endl;
        return Hj;
    }

    // Pre-compute distance and its exponents
    double d2 = px*px + py*py;
    double d = sqrt(d2);
    double d3 = d*d2;

    // Compute the Jacobian
    Hj <<   px/d, py/d, 0, 0,
            -py/d2, px/d2, 0, 0,
            py*(vx*py - vy*px)/d3, px*(vy*px - vx*py)/d3, px/d, py/d;

    return Hj;
}

void check_arguments(int argc, char* argv[]) {
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/infile.txt output.txt";

    bool has_valid_args = false;

    if (argc == 1)
        cerr << usage_instructions << endl;
    else if (argc == 2)
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    else if (argc == 3)
        has_valid_args = true;
    else
        cerr << "Too many arguments.\n" << usage_instructions << endl;

    if (!has_valid_args)
        exit(EXIT_FAILURE);
}

void check_files(ifstream &infile, string &in_name, ofstream &outfile, string &out_name) {
    if (!infile.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!outfile.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}
