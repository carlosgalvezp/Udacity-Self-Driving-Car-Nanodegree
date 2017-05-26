#include "mpc_model.h"

MPC_Model::MPC_Model(const Eigen::VectorXd& trajectory):
    trajectory_(trajectory)
{
}

void MPC_Model::operator()(ADvector& fg, const ADvector& x)
{
//    const double cost = 0.0;

//    // Compute cost over horizon
//    for (std::size_t i = 0U; i < kHorizonSteps; ++i)
//    {
//        // Compute heading error

//        // Cost function

//        // Constraints
//    }

//    fg[0] = cost;

//    // Set constraints g(x)
//    for (std::size_t t = 0; t < kHorizonSteps - 1U; ++t)
//    {
//        // Current state
//        const AD<double> x_t    = x[kIdxPx_start   + t];
//        const AD<double> y_t    = x[kIdxPy_start   + t];
//        const AD<double> psi_t  = x[kIdxPsi_start  + t];
//        const AD<double> v_t    = x[kIdxV_start    + t];
//        const AD<double> cte_t  = x[kIdxCTE_start  + t];
//        const AD<double> epsi_t = x[kIdxEpsi_start + t];

//        // Next state
//        const AD<double> x_t1    = x[kIdxPx_start   + t + 1];
//        const AD<double> y_t1    = x[kIdxPy_start   + t + 1];
//        const AD<double> psi_t1  = x[kIdxPsi_start  + t + 1];
//        const AD<double> v_t1    = x[kIdxV_start    + t + 1];
//        const AD<double> cte_t1  = x[kIdxCTE_start  + t + 1];
//        const AD<double> epsi_t1 = x[kIdxEpsi_start + t + 1];

//        // Current actuators
//        const AD<double> delta_t  = x[kIdxDelta_start  + t];
//        const AD<double> acc_t    = x[kIdxAcc_start  + t];

//        // Constraints
//        fg[1 + kIdxPx_start + t] = x_t1 - (x_t + v_t * CppAD::cos(psi_t) * dt);
//        fg[1 + kIdxPy_start + t] = y_t1 - (y_t + v_t * CppAD::sin(psi_t) * dt);
//        fg[1 + kIdxPsi_start + t] = psi_t1 - (psi_t + (v_t/Lf) * delta_t * dt);
//        fg[1 + kIdxV_start + t] = v_t1 - (v_t + acc_t * dt);
//        fg[1 + kIdxCTE_start + t] = cte_t1 - (cte_t + v_t * CppAD::sin(epsi_t) * dt);
//        fg[1 + kIdxEpsi_start + t] = epsi_t1 - (epsi_t + (v_t/Lf) * delta_t * dt);
//    }
}
