#include "mpc_model.h"
#include "tools.h"

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

MPC_Model::MPC_Model(const Eigen::VectorXd& trajectory):
    trajectory_(trajectory)
{
}

void MPC_Model::operator()(ADvector& fg, const ADvector& x)
{
    const double cost = 0.0;

    // Compute cost over horizon
    for (std::size_t i = 0U; i < kHorizonSteps; ++i)
    {
        // Compute heading error

        // Cost function

        // Constraints
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
        const double cte_t = Tools::polyeval(trajectory_, x_t) - y_t;
        const double psi_des_t = CppAD::atan(3*coeffs[3]*coeffs[3] +
                                             2*coeffs[2] +
                                               coeffs[1]);
        const double epsi_t = psi_t - psi_des_t;

        // Vehicle model
        fg[2 + kIdxPx_start + t]   = x_t1    - (x_t + v_t * CppAD::cos(psi_t) * dt);
        fg[2 + kIdxPy_start + t]   = y_t1    - (y_t + v_t * CppAD::sin(psi_t) * dt);
        fg[2 + kIdxPsi_start + t]  = psi_t1  - (psi_t + (v_t/Lf) * delta_t * dt);
        fg[2 + kIdxV_start + t]    = v_t1    - (v_t + acc_t * dt);
        fg[2 + kIdxCTE_start + t]  = cte_t1  - (cte_t + v_t * CppAD::sin(epsi_t) * dt);
        fg[2 + kIdxEpsi_start + t] = epsi_t1 - (epsi_t + (v_t/Lf) * delta_t * dt);
    }
}
