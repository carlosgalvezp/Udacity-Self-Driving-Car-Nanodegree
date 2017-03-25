#include <iostream>
#include "ukf.h"
#include "tools.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(const std::size_t n_states)
    : n_states_(n_states),
      x_(Eigen::VectorXd::Zero(n_states)),
      P_(Eigen::MatrixXd::Constant(n_states, n_states, 1.0)),
      x_sig_pred_(),
      weights_()
{
    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
}

void UKF::generateSigmaPoints(const Eigen::VectorXd& x,
                              const Eigen::MatrixXd& P,
                              std::vector<Eigen::VectorXd>& x_sig)
{
    const std::size_t n_states = x.rows();
    const std::size_t n_sigma_pts = 2U * n_states + 1U;
    const std::size_t lambda = 3U - n_states;

    const Eigen::MatrixXd P_sqrt = Tools::sqrt(P);

    x_sig.resize(n_sigma_pts);

    x_sig[0] = x;

    for(std::size_t i = 0U; i < n_states; ++i)
    {
        x_sig[1U + i]             = x + (lambda + n_states) * P_sqrt.col(i);
        x_sig[1U + n_states + i]  = x - (lambda + n_states) * P_sqrt.col(i);
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::predict(const MotionModel& motion_model, const double delta_t)
{
    const Eigen::MatrixXd Q = motion_model.getQ();
    const std::size_t n_augmented = motion_model.getAugmentedSize();

    Eigen::VectorXd x_a = Eigen::VectorXd::Zero(n_augmented);
    Eigen::MatrixXd P_a = Eigen::MatrixXd::Zero(n_augmented, n_augmented);

    x_a.head(n_states_) = x_;
    P_a.topLeftCorner(n_states_, n_states_) = P_;
    P_a.bottomRightCorner(Q.rows(), Q.cols()) = Q;

    // Generate sigma points
    generateSigmaPoints(x_a, P_a, x_sig_pred_);

    // Predict them
    for (std::size_t i = 0U; i < x_sig_pred_.size(); ++i)
    {
        x_sig_pred_[i] = motion_model.predict(x_sig_pred_[i], delta_t);
    }

    // Compute weights

    // Compute predicted mean and covariance matrix

    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
}

void UKF::update(const MeasurementModel& sensor_model, const Eigen::VectorXd& z)
{
    (void) sensor_model;
    (void) z;
}

