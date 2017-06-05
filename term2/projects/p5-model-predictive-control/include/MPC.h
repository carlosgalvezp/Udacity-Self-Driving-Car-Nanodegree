#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Eigen/Core>

#include "actuators.h"
#include "optimizer.h"

class MPC
{
public:
    MPC();

    /// \brief Solves the MPC model, outputting actuator commands
    /// \param state current state of the car, in local coordinates
    /// \param des_trajectory_coeffs desired trajectory, in local coordinates
    /// \param actuator_commands output steering and accelerator
    /// \param output_trajectory_x x coordinates of the predicted trajectory (local coord)
    /// \param output_trajectory_y y coordinates of the predicted trajectory (local coord)
    void computeCommands(const Eigen::VectorXd& state,
                         const Eigen::VectorXd& des_trajectory_coeffs,
                         Actuators& actuator_commands,
                         std::vector<double>& output_trajectory_x,
                         std::vector<double>& output_trajectory_y);

private:
    Optimizer optimizer_;
};

#endif  // MPC_H
