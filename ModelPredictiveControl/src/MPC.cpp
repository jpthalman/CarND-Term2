#include <iostream>
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <chrono>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

const int   N = 8;
const float dt = 0.1,
            Lf = 2.67;

const float ref_cte = 0,
            ref_epsi = 0,
            ref_v = 40;

const int   idx_x = 0,
            idx_y = idx_x + N,
            idx_psi = idx_y + N,
            idx_v = idx_psi + N,
            idx_cte = idx_v + N,
            idx_epsi = idx_cte + N,
            idx_delta = idx_epsi + N,
            idx_a = idx_delta + N - 1;

class FG_eval
{
public:
    using ADvector = CPPAD_TESTVECTOR(AD<double>);

    FG_eval(Eigen::VectorXd init_coeffs)
    {
        coeffs_ = init_coeffs;
    }

    void operator()(ADvector& fg, const ADvector& vars)
    {
        fg[0] = 0;

        for (int i = 0; i < N; ++i)
        {
            // the part of the cost based on the reference state
            fg[0] += 2100.0 * CppAD::pow(vars[idx_cte + i] - ref_cte, 2);
            fg[0] += 1900.0 * CppAD::pow(vars[idx_epsi + i] - ref_epsi, 2);
            fg[0] += 2.0 * CppAD::pow(vars[idx_v + i] - ref_v, 2);

            // minimize the use of actuators
            if (i < N - 1)
            {
                fg[0] += 3.0 * CppAD::pow(vars[idx_delta + i], 2);
                fg[0] += 3.0 * CppAD::pow(vars[idx_a + i], 2);
            }

            // minimize the value gap between sequential actuations
            if (i < N - 2)
            {
                fg[0] += 490.0 * CppAD::pow(vars[idx_delta + i + 1] - vars[idx_delta + i], 2);
                fg[0] += 4.8 * CppAD::pow(vars[idx_a + i + 1] - vars[idx_a + i], 2);
            }
        }

        // Initial constraints
        fg[1 + idx_x] = vars[idx_x];
        fg[1 + idx_y] = vars[idx_y];
        fg[1 + idx_psi] = vars[idx_psi];
        fg[1 + idx_v] = vars[idx_v];
        fg[1 + idx_epsi] = vars[idx_epsi];
        fg[1 + idx_cte] = vars[idx_cte];

        // the reset of the constraints
        for (int i = 0; i < N - 1; ++i)
        {
            // The state at time t+1 .
            AD<double> x1 = vars[idx_x + i + 1];
            AD<double> y1 = vars[idx_y + i + 1];
            AD<double> psi1 = vars[idx_psi + i + 1];
            AD<double> v1 = vars[idx_v + i + 1];
            AD<double> cte1 = vars[idx_cte + i + 1];
            AD<double> epsi1 = vars[idx_epsi + i + 1];

            // The state at time t.
            AD<double> x0 = vars[idx_x + i];
            AD<double> y0 = vars[idx_y + i];
            AD<double> psi0 = vars[idx_psi + i];
            AD<double> v0 = vars[idx_v + i];
            AD<double> cte0 = vars[idx_cte + i];
            AD<double> epsi0 = vars[idx_epsi + i];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[idx_delta + i];
            AD<double> a0 = vars[idx_a + i];

            AD<double> f0 = coeffs_[0] + coeffs_[1] * x0 + coeffs_[2]*x0*x0 + coeffs_[3]*x0*x0*x0;
            AD<double> psides0 = CppAD::atan(3*coeffs_[3] * x0 * x0 + 2 * coeffs_[2] * x0 + coeffs_[1]);

            fg[2 + idx_x + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + idx_y + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + idx_psi + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[2 + idx_v + i] = v1 - (v0 + a0 * dt);
            fg[2 + idx_cte + i] =
                    cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + idx_epsi + i] =
                    epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
        }
    }

private:
    Eigen::VectorXd coeffs_;
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double
            x = state[0],
            y = state[1],
            psi = state[2],
            v = state[3],
            cte = state[4],
            epsi = state[5];

    size_t n_vars = N * 6 + (N - 1) * 2;
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; ++i)
        vars[i] = 0;

    //load independent variables to current values
    vars[idx_x] = x;
    vars[idx_y] = y;
    vars[idx_psi] = psi;
    vars[idx_v] = v;
    vars[idx_cte] = cte;
    vars[idx_epsi] = epsi;

    // lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // set all non-actuator bounds to max
    for (int i = 0; i < idx_delta; ++i)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // upper and lower bounds for delta are set to 25 radians
    const double max_delta = 25 * M_PI / 180;
    for (int i = idx_delta; i < idx_a; ++i)
    {
        vars_lowerbound[i] = -max_delta;
        vars_upperbound[i] = max_delta;
    }

    // set acceleration bounds to 1
    for (int i = idx_a; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[idx_x] = x;
    constraints_lowerbound[idx_y] = y;
    constraints_lowerbound[idx_psi] = psi;
    constraints_lowerbound[idx_v] = v;
    constraints_lowerbound[idx_cte] = cte;
    constraints_lowerbound[idx_epsi] = epsi;

    constraints_upperbound[idx_x] = x;
    constraints_upperbound[idx_y] = y;
    constraints_upperbound[idx_psi] = psi;
    constraints_upperbound[idx_v] = v;
    constraints_upperbound[idx_cte] = cte;
    constraints_upperbound[idx_epsi] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    bool ok = true;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;

    vector<double> retValue;
    retValue.push_back(solution.x[idx_delta]);
    retValue.push_back(solution.x[idx_a]);
    for(int i=0;i<N-1;i++)
    {
        retValue.push_back(solution.x[idx_x+i]);
        retValue.push_back(solution.x[idx_y+i]);
    }
    return retValue;
}
