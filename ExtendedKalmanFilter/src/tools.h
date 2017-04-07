//
// Created by jacob on 3/28/2017.
//

#ifndef EXTENDEDKALMANFILTER_TOOLS_H
#define EXTENDEDKALMANFILTER_TOOLS_H

#include <vector>
#include <string>
#include <fstream>
#include "Eigen/Dense"

using kVectorList = const std::vector<Eigen::VectorXd>;

/**
 * Transforms a vector in polar coordinates to cartesian coordinates
 * */
Eigen::VectorXd polar_to_cartesian(const Eigen::VectorXd &v);

/**
 * Transforms a vector in cartesian coordinates to polar coordinates
 * */
Eigen::VectorXd cartesian_to_polar(const Eigen::VectorXd &z);

/**
 * Given a list of the estimations from the EKF, and a list of the corresponding ground truths, calcalates
 * the Root Mean Squared Error (RMSE) for each element of the vectors.
 * */
Eigen::VectorXd calculate_rmse(kVectorList &estimations, kVectorList &ground_truths);

/**
 * Given a radar sensor reading in polar coordinates, calculates the Jacobian matrix (Hj).
 * */
Eigen::MatrixXd calculate_jacobian(const Eigen::VectorXd &z);

/**
 * Check command line arguments to see if they are valid.
 * */
void check_arguments(int argc, char* argv[]);

/**
 * Check input and output files to see if they are valid.
 * */
void check_files(std::ifstream &infile, std::string &in_name, std::ofstream &outfile, std::string &out_name);

#endif //EXTENDEDKALMANFILTER_TOOLS_H
