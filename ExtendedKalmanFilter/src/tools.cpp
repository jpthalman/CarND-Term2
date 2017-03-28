//
// Created by jacob on 3/28/2017.
//

#include <iostream>
#include "tools.h"

using namespace Eigen;

VectorXd CalculateRMSE(kVectorList &estimations, kVectorList &ground_truths) {
    size_t n_obs = estimations.size();
    Vector4d rmse;
    rmse << 0, 0, 0, 0;

    if (n_obs == 0) {
        std::cerr << "Tools::CalculateRMSE | Number of estimations is zero." << std::endl;
        return rmse;
    }
    if (n_obs != ground_truths.size()) {
        std::cerr << "Tools::CalculateRMSE | Number of Estimations != Number of Ground Truths." << std::endl;
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

MatrixXd CalculateJacobian(const Vector4d &z) {
    MatrixXd Hj;

    double px = z[0];
    double py = z[1];
    double vx = z[2];
    double vy = z[3];

    // Don't divide by zero.
    if (fabs(px + py) < 1e-5) {
        std::cerr << "Tools::CalculateJacobian | Divide by zero error." << std::endl;
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
