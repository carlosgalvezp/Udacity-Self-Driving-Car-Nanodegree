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
    const double v        = x(2);
    const double yaw      = x(3);
    const double yaw_d    = x(4);
    const double nu_a     = x(5);
    const double nu_yawdd = x(6);

    const double dt2 = delta_t * delta_t;

    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(5);  // TODO: magic number

    // Predict
    if (Tools::isNotZero(yaw_d))
    {
        const double yaw_new = yaw + yaw_d * delta_t;
        x_pred(0) = x(0) + (v/yaw_d) * ( std::sin(yaw_new) - std::sin(yaw));
        x_pred(1) = x(1) + (v/yaw_d) * (-std::cos(yaw_new) + std::cos(yaw));
    }
    else
    {
        x_pred(0) = x(0) + v * delta_t * std::cos(yaw);
        x_pred(1) = x(1) + v * delta_t * std::sin(yaw);
    }
    x_pred(2) = x(2);
    x_pred(3) = x(3) + yaw_d * delta_t;
    x_pred(4) = x(4);

    // Add noise
    x_pred(0) += 0.5 * nu_a * dt2 * std::cos(yaw);
    x_pred(1) += 0.5 * nu_a * dt2 * std::sin(yaw);
    x_pred(2) += nu_a * delta_t;
    x_pred(3) += 0.5 * nu_yawdd * dt2;
    x_pred(4) += nu_yawdd * delta_t;

    return x_pred;
}
