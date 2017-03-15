#include "motion_model.h"

MotionModel::MotionModel(std::size_t n_states):
    n_states_(n_states)
{
}

Eigen::VectorXd MotionModel::predict(const Eigen::VectorXd& state,
                                     const double delta_t) const
{
    const Eigen::MatrixXd F = getF(delta_t);
    return F * state;
}


Eigen::MatrixXd MotionModel::getF(const double delta_t) const
{
    Eigen::MatrixXd F(n_states_, n_states_);

    F << 1.0,   0.0,    delta_t,    0.0,
         0.0,   1.0,    0.0,        delta_t,
         0.0,   0.0,    1.0,        0.0,
         0.0,   0.0,    0.0,        1.0;

    return F;
}

Eigen::MatrixXd MotionModel::getQ(const double delta_t) const
{
    Eigen::MatrixXd Q(n_states_, n_states_);

    const double dt2   = delta_t * delta_t;
    const double dt3_2 = dt2 * delta_t / 2.0;
    const double dt4_4 = dt3_2 * delta_t / 2.0;

    Q << dt4_4 * noise_ax_, 0.0,               dt3_2 * noise_ax_, 0.0,
         0.0,               dt4_4 * noise_ay_, 0.0,               dt3_2 * noise_ay_,
         dt3_2 * noise_ax_, 0.0,               dt2 * noise_ax_,   0.0,
         0.0,               dt3_2 * noise_ay_, 0.0,               dt2 * noise_ay_;

    return Q;
}

