#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include "mpc_model.h"
#include "tools.h"

class Optimizer
{
public:
    Optimizer();

    typedef CPPAD_TESTVECTOR(double) Dvector;

    Dvector solve(const Eigen::VectorXd& x,
                  MPC_Model &mpc_model);

    // MPC Horizon definition
    static constexpr std::size_t kHorizonSteps = 25U;
    static constexpr float       kDeltaT       = 0.05;  // [s]

    // Number of states: [x, y, psi, v, CTE, epsi]
    static constexpr std::size_t kNrStates     = 6U;

    // Number of actuators [delta, acceleration]
    static constexpr std::size_t kNrActuators  = 2U;

    // We have N states and (N-1) actuators, where N = kHorizonSteps
    static constexpr std::size_t kNrVars = kHorizonSteps * kNrStates +
                                           (kHorizonSteps - 1U) * kNrActuators;
    static constexpr std::size_t kNrConstraints = kHorizonSteps * kNrStates;

    // Indices to interesting variables in the vectors
    static constexpr std::size_t kIdxPx_start    = 0U;
    static constexpr std::size_t kIdxPx_end      = kIdxPx_start    + kHorizonSteps;

    static constexpr std::size_t kIdxPy_start    = kIdxPx_end;
    static constexpr std::size_t kIdxPy_end      = kIdxPy_start    + kHorizonSteps;

    static constexpr std::size_t kIdxPsi_start   = kIdxPy_end;
    static constexpr std::size_t kIdxPsi_end     = kIdxPsi_start   + kHorizonSteps;

    static constexpr std::size_t kIdxV_start     = kIdxPsi_end;
    static constexpr std::size_t kIdxV_end       = kIdxV_start     + kHorizonSteps;

    static constexpr std::size_t kIdxCTE_start   = kIdxV_end;
    static constexpr std::size_t kIdxCTE_end     = kIdxCTE_start   + kHorizonSteps;

    static constexpr std::size_t kIdxEpsi_start  = kIdxCTE_end;
    static constexpr std::size_t kIdxEpsi_end    = kIdxEpsi_start  + kHorizonSteps;

    static constexpr std::size_t kIdxDelta_start = kIdxEpsi_end;
    static constexpr std::size_t kIdxDelta_end   = kIdxDelta_start + kHorizonSteps - 1U;

    static constexpr std::size_t kIdxAcc_start   = kIdxDelta_end;
    static constexpr std::size_t kIdxAcc_end     = kIdxAcc_start   + kHorizonSteps - 1U;

    // Actuator limitations
    static constexpr double kMaxSteering = Tools::deg2rad(25.0);  // [rad]
    static constexpr double kMaxAcc      = 1.0;                   // [m/s^2]
private:

    // Optimizer workspace
    std::string options_;

    Dvector variables_;
    Dvector variables_lowerbound_;
    Dvector variables_upperbound_;

    Dvector constraints_lowerbound_;
    Dvector constraints_upperbound_;

    // Helper functions
    void setupOptions();
    void setupVariablesBounds();
    void setupConstraintsBounds();

    void updateInitialState(const Eigen::VectorXd& state, const double CTE, const double epsi);
};

#endif // OPTIMIZER_H
