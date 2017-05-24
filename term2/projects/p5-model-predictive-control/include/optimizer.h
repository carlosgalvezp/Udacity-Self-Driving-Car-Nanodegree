#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include "mpc_model.h"

class Optimizer
{
public:
    Optimizer();

    typedef CPPAD_TESTVECTOR(double) Dvector;

    Dvector solve(const Eigen::VectorXd& x,
                  MPC_Model &mpc_model);
};

#endif // OPTIMIZER_H
