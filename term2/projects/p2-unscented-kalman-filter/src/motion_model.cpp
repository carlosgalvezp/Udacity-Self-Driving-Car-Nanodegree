#include "motion_model.h"
#include <cmath>
#include "tools.h"

MotionModel::MotionModel():
    Q_(Eigen::MatrixXd::Zero(kNoiseVectorSize, kNoiseVectorSize))
{
    Q_(0,0) = std_a_ * std_a_;
    Q_(1,1) = std_yawdd_ * std_yawdd_;
}

Eigen::VectorXd MotionModel::predict(const Eigen::VectorXd& x,
                                     const double delta_t) const
{
    const double v        = x[2];
    const double yaw      = x[3];
    const double yaw_d    = x[4];
    const double nu_a     = x[5];
    const double nu_yawdd = x[6];

    const double dt2 = delta_t * delta_t;

    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(x.rows());

    if (Tools::isNotZero(yaw_d))
    {
        x_pred[0] = x[0] + (v/yaw_d)*(std::sin(yaw + yaw_d * delta_t) - std::sin(yaw))
                         + 0.5 * dt2 * std::cos(yaw) * nu_a;
        x_pred[1] = x[1] + (v/yaw_d)*(-std::cos(yaw + yaw_d * delta_t) + std::cos(yaw))
                         + 0.5 * dt2 * std::sin(yaw) * nu_a;
        x_pred[2] = x[2] + delta_t * nu_a;
        x_pred[3] = x[3] + yaw_d * delta_t + 0.5 * dt2 * nu_yawdd;
        x_pred[4] = x[4] + delta_t * nu_yawdd;
    }
    else
    {
        x_pred[0] = x[0] + v*std::cos(yaw)*delta_t + 0.5 * dt2 * std::cos(yaw) * nu_a;
        x_pred[1] = x[1] + v*std::sin(yaw)*delta_t + 0.5 * dt2 * std::sin(yaw) * nu_a;
        x_pred[2] = x[2] + delta_t * nu_a;
        x_pred[3] = x[3] + 0.5 * dt2 * nu_yawdd;
        x_pred[4] = x[4] + delta_t * nu_yawdd;
    }

    return x_pred;
}
