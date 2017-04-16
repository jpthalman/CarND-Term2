//
// Created by jaket on 4/15/2017.
//

#include "tools.h"

Eigen::VectorXd polar_to_cartesian(Eigen::VectorXd &polar) {
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