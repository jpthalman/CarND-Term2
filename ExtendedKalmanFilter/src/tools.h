//
// Created by jacob on 3/28/2017.
//

#ifndef EXTENDEDKALMANFILTER_TOOLS_H
#define EXTENDEDKALMANFILTER_TOOLS_H

#include <vector>
#include "Eigen/Dense"

using kVectorList = const std::vector<Eigen::VectorXd>;

/**
 * Given a list of the estimations from the EKF, and a list of the corresponding ground truths, calcalates
 * the Root Mean Squared Error (RMSE) for each element of the vectors.
 * */
Eigen::VectorXd CalculateRMSE(kVectorList &estimations, kVectorList &ground_truths);

/**
 * Given a radar sensor reading in polar coordinates, calculates the Jacobian matrix (Hj).
 * */
Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &z);

#endif //EXTENDEDKALMANFILTER_TOOLS_H
