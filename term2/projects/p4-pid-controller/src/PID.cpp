#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(const double kp, const double kd, const double ki):
    kp_(kp),
    ki_(ki),
    kd_(kd),
    p_error_(0.0),
    i_error_(0.0),
    d_error_(0.0)
{}

PID::~PID() {}

void PID::UpdateError(double cte)
{
}

double PID::TotalError()
{
}

