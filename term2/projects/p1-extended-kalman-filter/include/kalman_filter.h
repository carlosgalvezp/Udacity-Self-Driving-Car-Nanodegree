#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "motion_model.h"
#include "measurement_model.h"

// Initial state
static const double x0_ = 1.0E-3;  // To avoid dividing by zero

// Initial uncertainty
static const double p0_ = 1.0E3;

class KalmanFilter
{
public:
    KalmanFilter(const std::size_t state_dimension);

    void setState(const Eigen::VectorXd& x) { x_ = x; }

    /// \brief Predicts the state and the state covariance using
    ///        the process model
    /// \param motion_model motion model used for prediction
    /// \param delta_t difference in time w.r.t. previous iteration, in seconds
    ///
    void predict(const MotionModel& motion_model, const double delta_t);

    /// \brief Updates the state by using the Extended Kalman Filter equations
    /// \param sensor_model the measurement model
    /// \param z the measurement
    void update(const MeasurementModel& sensor_model, const Eigen::VectorXd& z);


    /// \brief Returns the current state
    /// \return the current state
    const Eigen::VectorXd& getState() const { return x_; }

    /// \brief Returns the  current covariance matrix
    /// \return the current covariance matrix
    const Eigen::MatrixXd& getCovariance() const { return P_; }

private:
    // State vector
    Eigen::VectorXd x_;

    // State covariance matrix
    Eigen::MatrixXd P_;

    // Identity matrix
    Eigen::MatrixXd I_;


};

#endif /* KALMAN_FILTER_H_ */
