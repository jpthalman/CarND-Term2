//
// Created by jacob on 3/28/2017.
//

#include <iostream>
#include "tools.h"

using namespace std;
using namespace Eigen;

VectorXd polar_to_cartesian(const VectorXd &v) {
    double rho = v(0);
    double phi = v(1);
    double rho_dot = v(2);

    VectorXd cartesian = Vector4d::Zero();
    cartesian <<    rho * cos(phi),  // px
                    rho * sin(phi), // py
                    rho_dot * cos(phi), // vx
                    rho_dot * sin(phi); //vy
    return cartesian;
}

VectorXd calculate_rmse(kVectorList &estimations, kVectorList &ground_truths) {
    size_t n_obs = estimations.size();
    Vector4d rmse = Vector4d::Zero();

    // error handling
    if (n_obs == 0) {
        cerr << "calculate_rmse | Number of estimations is zero." << endl;
        return rmse;
    }
    if (n_obs != ground_truths.size()) {
        cerr << "calculate_rmse | Number of Estimations != Number of Ground Truths." << endl;
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
    Hj <<   0.,0.,0.,0.,
            0.,0.,0.,0.,
            0.,0.,0.,0.;

    VectorXd cartesian = polar_to_cartesian(z);

    double px = cartesian(0);
    double py = cartesian(1);
    double vx = cartesian(2);
    double vy = cartesian(3);

    // Don't divide by zero.
    if (abs(px) + abs(py) < 1e-5) {
        cerr << "calculate_jacobian | Divide by zero error." << endl;
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
