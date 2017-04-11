#ifndef UKF_H
#define UKF_H
#include <Eigen/Dense>
#include <vector>

#include "measurement_package.h"
#include "motion_model.h"
#include "measurement_model.h"

static const double kInitialUncertainty = 1.0F;

/// \brief Implements the Unscented Kalman Filter (UKF)
class UKF
{
public:
    /// \brief UKF
    /// \param motion_model reference to the motion model used
    UKF(const MotionModel& motion_model);

    /// \brief returns the current state
    /// \return the current state
    const Eigen::VectorXd& getState() const { return x_; }

    /// \brief sets the state to a new value
    /// \param x new state
    void setState(const Eigen::VectorXd& x) { x_ = x; }

    /// \brief predict step of the UKF
    /// \param motion_model
    /// \param delta_t time difference (seconds) w.r.t. previous update
    void predict(const MotionModel& motion_model, const double delta_t);

    /// \brief update step of the UKF
    /// \param sensor_model sensor model that can predict measurements
    /// \param z current measurement
    /// \return the NIS of the used sensor
    double update(const MeasurementModel& sensor_model, const Eigen::VectorXd& z);
private:
    // Dimension of the state vector
    const std::size_t n_states_;

    // State vector: [pos_x pos_y velocity yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    // State covariance matrix
    Eigen::MatrixXd P_;

    // Motion model
    const MotionModel& motion_model_;

    // Predicted sigma points
    std::vector<Eigen::VectorXd> x_sig_pred_;

    // Lambda parameter
    const double lambda_;

    // Weights of sigma points
    std::vector<double> weights_;

    void generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                             std::vector<Eigen::VectorXd>& x_sig);

    std::size_t computeNumberOfSigmaPoints(const std::size_t n_states) const;
};

#endif /* UKF_H */
