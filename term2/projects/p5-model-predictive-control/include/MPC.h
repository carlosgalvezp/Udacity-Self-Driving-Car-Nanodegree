#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Eigen/Core>

#include "actuators.h"

class MPC
{
public:
    /// \brief Solves the model
    /// \param state current state of the car
    /// \param coeffs desired trajectory
    /// \return actuator commands for next control loop
    Actuators solve(const Eigen::VectorXd& state,
                    const Eigen::VectorXd& coeffs);
};

#endif  // MPC_H
