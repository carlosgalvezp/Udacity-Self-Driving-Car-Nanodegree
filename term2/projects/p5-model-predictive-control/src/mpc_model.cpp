#include "mpc_model.h"

MPC_Model::MPC_Model(const Eigen::VectorXd& trajectory):
    trajectory_(trajectory)
{
}

void MPC_Model::operator()(ADvector& fg, const ADvector& x)
{
    const double cost = 0.0;
    const std::size_t N = 10U;

    // Compute next state over horizon
    for (std::size_t i = 0U; i < N; ++i)
    {

    }

    // Compute cost over horizon
    for (std::size_t i = 0U; i < N; ++i)
    {
        // Compute heading error

        // Cost function

        // Constraints
    }

    fg[0] = cost;
}
