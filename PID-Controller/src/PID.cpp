#include <ctime>
#include <cmath>
#include <iostream>
#include "PID.h"

PID::PID(double target, double kp_init, double ki_init, double kd_init) :
        set_point_(target), Kp_(kp_init), Ki_(ki_init), Kd_(kd_init)
{
    // initialize errors to zero
    p_error_ = i_error_ = d_error_ = 0;

    // initialize timestamp to zero
    prev_timestamp_ = 0;

    // set max integral to a really big value
    max_integral_ = 1e9;
}

PID::~PID() {}

double PID::Update(double cte)
{
    std::time_t current_timestamp = time(nullptr);

    // If this is the first observation, do not scale the derivative (AKA divide by 1).
    // Otherwise, calculate the elapsed time in seconds.
    double dt = ((current_timestamp != 0) ? (current_timestamp - prev_timestamp_) / 1e6 : 1.0);

    p_error_ = set_point_ - cte;
    d_error_ = (p_error_ - prev_error_);
    i_error_ += p_error_;

    std::cout << "P:" << p_error_ << " I:" << i_error_ << " D:" << dt << std::endl;

    // prevent integral windup
    if (fabs(i_error_) > max_integral_)
        // multiply max_integral_ by the sign of the current integral
        i_error_ = max_integral_ * ((i_error_ > 0) ? 1.0 : ((i_error_ < 0) ? -1.0 : 0.0));

    prev_error_ = p_error_;
    prev_timestamp_ = current_timestamp;

    return Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
}
