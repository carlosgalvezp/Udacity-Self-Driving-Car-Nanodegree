#include "PID.h"
#include <iostream>

PID::PID(const double kp, const double ki, const double kd):
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
    double output = -(kp_ * p_error_ +
                      ki_ * i_error_ +
                      kd_ * d_error_);

    if (output > kSteeringMax)
    {
        std::cout << "[WARNING] Too large steering output: " << output << std::endl;
        output = kSteeringMax;
    }
    else if (output < kSteeringMin)
    {
        std::cout << "[WARNING] Too large steering output: " << output << std::endl;
        output = kSteeringMin;
    }

    return output;
}

