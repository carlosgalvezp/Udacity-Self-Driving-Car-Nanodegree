#include "PID.h"

PID::PID(const double kp, const double kd, const double ki):
    kp_(kp),
    ki_(ki),
    kd_(kd),
    p_error_(0.0),
    d_error_(0.0),
    i_error_(0.0)
{}

PID::~PID() {}

void PID::updateError(double cte)
{
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;
}

double PID::computeSteering()
{
    return kp_ * p_error_ + ki_ * i_error_ + kd_ * d_error_;
}

