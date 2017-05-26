#ifndef MPC_MODEL_H
#define MPC_MODEL_H

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

/// \brief The MPC_Model class
///
/// Reference:
/// https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm
class MPC_Model
{
public:
    explicit MPC_Model(const Eigen::VectorXd& trajectory);

    /// Required
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    /// \brief operator (), required per definition of FG_eval
    /// \param fg f and g functions, N+1-dimensional ADvector
    ///           fg[0] = f
    ///           fg[1] = g_1
    ///           fg[2] = g_2
    ///           ...
    ///           fg[N] = g_N
    /// \param x state on which f(x) and g(x) are evaluated
    void operator()(ADvector& fg, const ADvector& x);

private:
    const Eigen::VectorXd& trajectory_;
};

#endif  // MPC_MODEL_H
