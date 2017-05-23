#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Eigen/Core>

class MPC
{
public:
    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    std::vector<double> solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif  // MPC_H
