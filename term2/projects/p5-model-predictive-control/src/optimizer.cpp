#include "optimizer.h"

#include <cppad/ipopt/solve.hpp>

Optimizer::Optimizer():
    options_(""),
    variables_(kNrVars),
    variables_lowerbound_(kNrVars),
    variables_upperbound_(kNrVars),
    constraints_lowerbound_(kNrConstraints),
    constraints_upperbound_(kNrConstraints)
{
    setupOptions();
    setupVariablesBounds();
    setupConstraintsBounds();
}

Optimizer::Dvector Optimizer::solve(const Eigen::VectorXd &state,
                                    Optimizer::MPC_Model& model)
{
    // Setup initial state
    updateInitialState(state);

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, MPC_Model>(
                options_,
                variables_, variables_lowerbound_, variables_upperbound_,
                constraints_lowerbound_, constraints_upperbound_,
                model, solution);

    // Check some of the solution values
    if (solution.status == CppAD::ipopt::solve_result<Dvector>::success)
    {
        std::cout << "[WARNING] solution.status != SUCCESS";
    }

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    return solution.x;
}

void Optimizer::setupOptions()
{
    // Uncomment this if you'd like more print information
    options_ += "Integer print_level  0\n";

    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options_ += "Sparse  true        forward\n";
    options_ += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options_ += "Numeric max_cpu_time          0.5\n";
}

void Optimizer::setupVariablesBounds()
{
    // State variables can have any value
    for (std::size_t i = 0U; i < kIdxDelta_start; ++i)
    {
        variables_lowerbound_[i] = std::numeric_limits<double>::lowest();
        variables_upperbound_[i] = std::numeric_limits<double>::max();
    }

    // Steering limitations [rad]
    for (std::size_t i = kIdxDelta_start; i < kIdxDelta_end; ++i)
    {
        variables_lowerbound_[i] = -kMaxSteering;
        variables_upperbound_[i] =  kMaxSteering;
    }

    // Acceleration limitations [m/s^2]
    for (std::size_t i = kIdxAcc_start; i < kIdxAcc_end; ++i)
    {
        variables_lowerbound_[i] = -kMaxAcc;
        variables_upperbound_[i] =  kMaxAcc;
    }
}

void Optimizer::setupConstraintsBounds()
{
    // Do nothing - the lower and upper bounds are equal to 0.0, since
    // the equations for the constraints are in the form: g(x) = 0.0
}

void Optimizer::updateInitialState(const Eigen::VectorXd& state)
{
    // Retrieve data
    const double px   = state[0U];
    const double py   = state[1U];
    const double psi  = state[2U];
    const double v    = state[3U];
    const double cte  = state[4U];
    const double epsi = state[5U];

    // Setup the initial state at t = 0
    variables_[kIdxPx_start]   = px;
    variables_[kIdxPy_start]   = py;
    variables_[kIdxPsi_start]  = psi;
    variables_[kIdxV_start]    = v;
    variables_[kIdxCTE_start]  = cte;
    variables_[kIdxEpsi_start] = epsi;

    // Setup the upper and lower bounds of the state at t = 0 to the state
    // i.e. it cannot be modififed
    variables_lowerbound_[kIdxPx_start]   = px;
    variables_lowerbound_[kIdxPy_start]   = py;
    variables_lowerbound_[kIdxPsi_start]  = psi;
    variables_lowerbound_[kIdxV_start]    = v;
    variables_lowerbound_[kIdxCTE_start]  = cte;
    variables_lowerbound_[kIdxEpsi_start] = epsi;

    variables_upperbound_[kIdxPx_start]   = px;
    variables_upperbound_[kIdxPy_start]   = py;
    variables_upperbound_[kIdxPsi_start]  = psi;
    variables_upperbound_[kIdxV_start]    = v;
    variables_upperbound_[kIdxCTE_start]  = cte;
    variables_upperbound_[kIdxEpsi_start] = epsi;
}

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

Optimizer::MPC_Model::MPC_Model(const Eigen::VectorXd& trajectory):
    trajectory_(trajectory)
{
}

void Optimizer::MPC_Model::operator()(ADvector& fg, const ADvector& x)
{
    CppAD::AD<double> cost = 0.0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < kHorizonSteps; ++i)
    {
        cost += CppAD::pow(x[kIdxCTE_start + i]  - kRefCte,  2);
        cost += CppAD::pow(x[kIdxEpsi_start + i] - kRefEpsi, 2);
        cost += CppAD::pow(x[kIdxV_start + i]    - kRefV,    2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < kHorizonSteps - 1; i++)
    {
        cost += CppAD::pow(x[kIdxDelta_start + i], 2);
        cost += CppAD::pow(x[kIdxAcc_start + i],   2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < kHorizonSteps - 2; i++)
    {
        cost += CppAD::pow(x[kIdxDelta_start + i + 1] - x[kIdxDelta_start + i], 2);
        cost += CppAD::pow(x[kIdxAcc_start   + i + 1] - x[kIdxAcc_start   + i], 2);
    }

    fg[0] = cost;

    // Set constraints g(x) = fg[1 : N]
    // g_0 = x, so to make sure the initial state does not change during optimization
    fg[1 + kIdxPx_start]   = x[kIdxPx_start];
    fg[1 + kIdxPy_start]   = x[kIdxPy_start];
    fg[1 + kIdxPsi_start]  = x[kIdxPsi_start];
    fg[1 + kIdxV_start]    = x[kIdxV_start];
    fg[1 + kIdxCTE_start]  = x[kIdxCTE_start];
    fg[1 + kIdxEpsi_start] = x[kIdxEpsi_start];

    // g_1:N, how the state changes over time
    for (std::size_t t = 0; t < kHorizonSteps - 1U; ++t)
    {
        // Current state
        const CppAD::AD<double> x_t    = x[kIdxPx_start   + t];
        const CppAD::AD<double> y_t    = x[kIdxPy_start   + t];
        const CppAD::AD<double> psi_t  = x[kIdxPsi_start  + t];
        const CppAD::AD<double> v_t    = x[kIdxV_start    + t];

        // Next state
        const CppAD::AD<double> x_t1    = x[kIdxPx_start   + t + 1];
        const CppAD::AD<double> y_t1    = x[kIdxPy_start   + t + 1];
        const CppAD::AD<double> psi_t1  = x[kIdxPsi_start  + t + 1];
        const CppAD::AD<double> v_t1    = x[kIdxV_start    + t + 1];
        const CppAD::AD<double> cte_t1  = x[kIdxCTE_start  + t + 1];
        const CppAD::AD<double> epsi_t1 = x[kIdxEpsi_start + t + 1];

        // Current actuators
        const CppAD::AD<double> delta_t  = x[kIdxDelta_start  + t];
        const CppAD::AD<double> acc_t    = x[kIdxAcc_start  + t];

        // Compute actual CTE and Epsi_t
        const CppAD::AD<double> fx_t  = trajectory_[0]                      +
                                        trajectory_[1] *            x_t     +
                                        trajectory_[2] * CppAD::pow(x_t, 2) +
                                        trajectory_[3] * CppAD::pow(x_t, 3);

        const CppAD::AD<double> cte_t = fx_t - y_t;
        const CppAD::AD<double> psi_des_t = CppAD::atan(    trajectory_[1] +
                                                        2 * trajectory_[2] +
                                                        3 * trajectory_[3] * trajectory_[3]);
        const CppAD::AD<double> epsi_t = psi_t - psi_des_t;

        // Vehicle model
        fg[2 + kIdxPx_start + t]   = x_t1    - (x_t + v_t * CppAD::cos(psi_t) * dt);
        fg[2 + kIdxPy_start + t]   = y_t1    - (y_t + v_t * CppAD::sin(psi_t) * dt);
        fg[2 + kIdxPsi_start + t]  = psi_t1  - (psi_t + (v_t/Lf) * delta_t * dt);
        fg[2 + kIdxV_start + t]    = v_t1    - (v_t + acc_t * dt);
        fg[2 + kIdxCTE_start + t]  = cte_t1  - (cte_t + v_t * CppAD::sin(epsi_t) * dt);
        fg[2 + kIdxEpsi_start + t] = epsi_t1 - (epsi_t + (v_t/Lf) * delta_t * dt);
    }
}
