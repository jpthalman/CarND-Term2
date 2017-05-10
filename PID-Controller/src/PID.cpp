#include <ctime>
#include <cmath>
#include <iostream>
#include "PID.h"

PID::PID(double target, double kp_init, double ki_init, double kd_init) :
        set_point_(target), Kp_(kp_init), Ki_(ki_init), Kd_(kd_init)
{
    // initialize errors to zero
    p_error_ = i_error_ = d_error_ = 0;

    // set max integral to a really big value
    max_integral_ = 1e9;
}

PID::~PID() {}

double PID::Update(const double cte)
{
    p_error_ = set_point_ - cte;
    d_error_ = (p_error_ - prev_error_);
    i_error_ += p_error_;

    // prevent integral windup
    if (fabs(i_error_) > max_integral_)
        // multiply max_integral_ by the sign of the current integral
        i_error_ = max_integral_ * ((i_error_ > 0) ? 1.0 : ((i_error_ < 0) ? -1.0 : 0.0));

//    std::cout << "P:" << p_error_ << " I:" << i_error_ << " D:" << d_error_ << std::endl;

    prev_error_ = p_error_;

    return Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
}

inline
void PID::resetTerms(const double kp_init, const double ki_init, const double kd_init)
{
    Kp_ = kp_init;
    Ki_ = ki_init;
    Kd_ = kd_init;
}
