#ifndef MEASUREMENT_MODEL_LIDAR_H
#define MEASUREMENT_MODEL_LIDAR_H

#include "measurement_model.h"

/// Laser measurement noise standard deviation - px
static const double std_px_ = 0.15;  // [m]^2

/// Laser measurement noise standard deviation - py
static const double std_py_ = 0.15;  // [m]^2

/// \brief Implements a Lidar sensor model
class MeasurementModelLidar : public MeasurementModel
{
public:
    /// \brief Constructor
    /// \param n_states dimension of the state vector
    explicit MeasurementModelLidar(std::size_t n_states);
    virtual ~MeasurementModelLidar();

    /// \brief computes the initialization state, given a measurement
    /// \param z initial measurement
    /// \return the initial state for the filter
    virtual Eigen::VectorXd computeInitialState(const Eigen::VectorXd& z) const;

    /// \brief Computes z_hat = h(x')
    /// \param state predicted state, x'
    /// \return the predicted measurement, z_hat
    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const;

    /// \brief Computes the difference between 2 measurements
    /// \param z_a first measurement
    /// \param z_b second measurement
    /// \return the difference y = z_a - z_b
    virtual Eigen::VectorXd computeDifference(const Eigen::VectorXd& z_a,
                                              const Eigen::VectorXd& z_b) const;

    /// \brief computes and returns the measurement noise matrix, R
    /// \return the R matrix
    virtual Eigen::MatrixXd getR() const { return R_; }

private:
    const std::size_t n_observed_states_ = 2U;
    Eigen::MatrixXd R_;
};

#endif // MEASUREMENT_MODEL_LIDAR_H
