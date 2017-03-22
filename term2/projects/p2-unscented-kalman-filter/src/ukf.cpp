#include <iostream>
#include "ukf.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(const std::size_t n_states)
{
    // initial state vector
    x_ = Eigen::VectorXd(n_states);

    // initial covariance matrix
    P_ = Eigen::MatrixXd(n_states, n_states);







    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage /*meas_package*/)
{
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double /*delta_t*/)
{
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage /*meas_package*/)
{
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage /*meas_package*/)
{
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
}
