#include "MPC.h"

// TODO: Set the timestep length and duration
size_t N = 0;
double dt = 0;

MPC::MPC()
    : optimizer_()
{
}

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

bool MPC::computeCommands(const Eigen::VectorXd& state,
                          const Eigen::VectorXd& des_trajectory_coeffs,
                          Actuators& actuator_commands,
                          std::vector<double>& output_trajectory_x,
                          std::vector<double>& output_trajectory_y)
{
    // Create MPC model given the trajectory
    MPC_Model model(des_trajectory_coeffs);

    // Solve the MPC problem
    const Optimizer::Dvector solution = optimizer_.solve(state, model);

    // Compute vehicle commands
    actuator_commands.steering     = solution[Optimizer::kIdxDelta_start];
    actuator_commands.acceleration = solution[Optimizer::kIdxAcc_start];

    // Compute predicted trajectory
    const std::size_t trajectory_size = Optimizer::kHorizonSteps;
    output_trajectory_x.resize(trajectory_size);
    output_trajectory_y.resize(trajectory_size);

    for (std::size_t i = 0U; i < trajectory_size; ++i)
    {
        output_trajectory_x[i] = solution[Optimizer::kIdxPx_start + i];
        output_trajectory_x[i] = solution[Optimizer::kIdxPy_start + i];
    }

    return true;
}
