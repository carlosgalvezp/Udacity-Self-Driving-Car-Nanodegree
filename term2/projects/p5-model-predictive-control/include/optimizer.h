#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "mpc_model.h"

class Optimizer
{
public:
    Optimizer();

    typedef CPPAD_TESTVECTOR(double) Dvector;

    Dvector solve(const DVector& x,
                  const MPC_Model& mpc_model);
};

#endif // OPTIMIZER_H
