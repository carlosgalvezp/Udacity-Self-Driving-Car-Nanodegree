#include "motion_model.h"

MotionModel::MotionModel(std::size_t state_dimension):
    state_dimension_(state_dimension)
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
    Eigen::MatrixXd F(state_dimension_, state_dimension_);

    F << 1.0,   0.0,    delta_t,    0.0,
         0.0,   1.0,    0.0,        delta_t,
         0.0,   0.0,    1.0,        0.0,
         0.0,   0.0,    0.0,        1.0;

    return F;
}

Eigen::MatrixXd MotionModel::getQ(const double delta_t) const
{
    Eigen::MatrixXd Q(state_dimension_, state_dimension_);

    const double dt2   = delta_t * delta_t;
    const double dt3_2 = dt2 * delta_t / 2.0;
    const double dt4_4 = dt3_2 * delta_t / 2.0;

    Q << dt4_4 * sigma_ax_, 0.0,               dt3_2 * sigma_ax_, 0.0,
         0.0,               dt4_4 * sigma_ay_, 0.0,               dt3_2 * sigma_ay_,
         dt3_2 * sigma_ax_, 0.0,               dt2 * sigma_ax_,   0.0,
         0.0,               dt3_2 * sigma_ay_, 0.0,               dt2 * sigma_ay_;

    return Q;
}

