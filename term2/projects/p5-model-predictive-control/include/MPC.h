#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Eigen/Core>

#include "actuators.h"
#include "optimizer.h"
#include "mpc_model.h"

class MPC
{
public:
    MPC();

    /// \brief Solves the MPC model, outputting actuator commands
    /// \param state current state of the car
    /// \param coeffs desired trajectory
    /// \return actuator commands for next control loop
    Actuators computeCommands(const Eigen::VectorXd& state,
                              const Eigen::VectorXd& trajectory);

private:
    Optimizer optimizer_;
};

#endif  // MPC_H
