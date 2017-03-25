#include <iostream>
#include "ukf.h"

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

void UKF::generateSigmaPoints()
{

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::predict(const MotionModel& motion_model, const double delta_t)
{
    (void) motion_model;
    (void) delta_t;

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

