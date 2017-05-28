#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Dense>
#include <cppad/cppad.hpp>

#include "tools.h"

class Optimizer
{
public:
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

        static constexpr std::size_t kHorizonSteps = 25;
        static constexpr double      dt    = 0.05;  // [s]

    private:
        const Eigen::VectorXd& trajectory_;
    };

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

    // Reference values
    static constexpr double kRefCte  = 0.0;   // [m]
    static constexpr double kRefEpsi = 0.0;   // [rad]
    static constexpr double kRefV    = 30.0;  // [m/s] TODO - make it adaptive with curve radius

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

    void updateInitialState(const Eigen::VectorXd& state);
};

#endif // OPTIMIZER_H
