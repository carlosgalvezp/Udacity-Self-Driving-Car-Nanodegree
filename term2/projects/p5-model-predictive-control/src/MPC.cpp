#include "MPC.h"

// TODO: Set the timestep length and duration
size_t N = 0;
double dt = 0;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

Actuators MPC::solve(const Eigen::VectorXd &state,
                     const Eigen::VectorXd &trajectory)
{
    const Solver::Solution solution = solver_.solve(state, trajectory);

    Actuators actuators = Actuators();

    actuators.acceleration = solution[0U];
    actuators.steering     = solution[1U];

    return actuators;
}
