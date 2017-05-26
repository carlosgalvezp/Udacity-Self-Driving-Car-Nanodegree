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
                                    MPC_Model& model)
{
    bool ok = true;
    size_t i;

    // Setup initial state

    // Setup bounds for initial state

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
//    Dvector vars(n_vars);

//    for (int i = 0; i < n_vars; i++)
//    {
//        vars[i] = 0;
//    }

//    Dvector vars_lowerbound(n_vars);
//    Dvector vars_upperbound(n_vars);
//    // TODO: Set lower and upper limits for variables.

//    // Lower and upper limits for the constraints
//    // Should be 0 besides initial state.
//    Dvector constraints_lowerbound(n_constraints);
//    Dvector constraints_upperbound(n_constraints);

//    for (int i = 0; i < n_constraints; i++)
//    {
//        constraints_lowerbound[i] = 0;
//        constraints_upperbound[i] = 0;
//    }

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, MPC_Model>(
                options_,
                variables_, variables_lowerbound_, variables_upperbound_,
                constraints_lowerbound_, constraints_upperbound_,
                model, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

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

}

void Optimizer::updateInitialState(const Eigen::VectorXd& state,
                                   const double CTE,
                                   const double epsi)
{
    // Retrieve data
    const double px  = state[0U];
    const double py  = state[1U];
    const double psi = state[2U];
    const double v   = state[3U];

    // Setup the initial state at t = 0
    variables_[kIdxPx_start]   = px;
    variables_[kIdxPy_start]   = py;
    variables_[kIdxPsi_start]  = psi;
    variables_[kIdxV_start]    = v;
    variables_[kIdxCTE_start]  = CTE;
    variables_[kIdxEpsi_start] = epsi;

    // Setup the upper and lower bounds of the state at t = 0 to the state
    // i.e. it cannot be modififed
    variables_lowerbound_[kIdxPx_start]   = px;
    variables_lowerbound_[kIdxPy_start]   = py;
    variables_lowerbound_[kIdxPsi_start]  = psi;
    variables_lowerbound_[kIdxV_start]    = v;
    variables_lowerbound_[kIdxCTE_start]  = CTE;
    variables_lowerbound_[kIdxEpsi_start] = epsi;

    variables_upperbound_[kIdxPx_start]   = px;
    variables_upperbound_[kIdxPy_start]   = py;
    variables_upperbound_[kIdxPsi_start]  = psi;
    variables_upperbound_[kIdxV_start]    = v;
    variables_upperbound_[kIdxCTE_start]  = CTE;
    variables_upperbound_[kIdxEpsi_start] = epsi;
}
