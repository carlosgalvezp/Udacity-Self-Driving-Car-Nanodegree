#include "MPC.h"

// TODO: Set the timestep length and duration
size_t N = 0;
double dt = 0;

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

/// This is a helper class required by CppAD::ipopt::solve.
/// Documentation on this function can be found here:
///
/// https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm
///
/// The purpose of this class is to define the multi-dimensional
/// functions f(x) and g(x), where x is the state whose
/// solution we intend to find.
class FG_eval
{
public:
    /// Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    /// Required
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    /// \brief operator (), required per definition of FG_eval
    /// \param fg f and g functions
    /// \param x state
    void operator()(ADvector& fg, const ADvector& x)
    {
        // TODO: implement MPC
        // fg a vector of constraints, x is a vector of constraints.
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
    }
};

Actuators MPC::solve(const Eigen::VectorXd &state,
                     const Eigen::VectorXd &trajectory)
{
    const Solver::Solution solution = solver_.solve(state, trajectory);

    Actuators actuators = Actuators();

    actuators.acceleration = solution[0U];
    actuators.steering     = solution[1U];

    return actuators;
}
