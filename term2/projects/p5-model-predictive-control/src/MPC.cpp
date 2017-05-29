#include "MPC.h"

MPC::MPC()
    : optimizer_()
{
}

bool MPC::computeCommands(const Eigen::VectorXd& state,
                          const Eigen::VectorXd& des_trajectory_coeffs,
                          Actuators& actuator_commands,
                          std::vector<double>& output_trajectory_x,
                          std::vector<double>& output_trajectory_y)
{
    // Create MPC model given the trajectory
    Optimizer::MPC_Model model(des_trajectory_coeffs);

    // Solve the MPC problem
    const Optimizer::Dvector solution = optimizer_.solve(state, model);

    // Extract vehicle commands
    actuator_commands.steering     = solution[Optimizer::kIdxDelta_start];
    actuator_commands.acceleration = solution[Optimizer::kIdxAcc_start];

    // Extract predicted trajectory
    const std::size_t trajectory_size = Optimizer::kHorizonSteps;
    output_trajectory_x.resize(trajectory_size);
    output_trajectory_y.resize(trajectory_size);

    for (std::size_t i = 0U; i < trajectory_size; ++i)
    {
        output_trajectory_x[i] = solution[Optimizer::kIdxPx_start + i];
        output_trajectory_y[i] = solution[Optimizer::kIdxPy_start + i];
    }

    return true;
}
