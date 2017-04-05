#ifndef MEASUREMENT_MODEL_RADAR_H
#define MEASUREMENT_MODEL_RADAR_H

#include "measurement_model.h"

/// Radar measurement noise standard deviation - range
static const double std_range_      = 0.3;   // [m]

/// Radar measurement noise standard deviation - bearing
static const double std_bearing_    = 0.03;  // [rad]

/// Radar measurement noise standard deviation - range-rate
static const double std_range_rate_ = 0.3;   // [m/s]

/// \brief Implements a radar sensor model
class MeasurementModelRadar : public MeasurementModel
{
public:
    /// \brief Constructor
    /// \param n_states dimension of the state vector
    explicit MeasurementModelRadar(const std::size_t n_states);
    virtual ~MeasurementModelRadar();

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

    /// \brief Computes and returns the measurement noise matrix, R
    /// \return the R matrix
    virtual Eigen::MatrixXd getR() const { return R_; }

private:
    const std::size_t n_observed_states_ = 3U;
    Eigen::MatrixXd R_;
};

#endif // MEASUREMENT_MODEL_RADAR_H
