#ifndef MEASUREMENT_MODEL_RADAR_H
#define MEASUREMENT_MODEL_RADAR_H

#include "measurement_model.h"

/// Radar measurement noise standard deviation - range
static const double noise_range_      = 0.09;   // [m]^2

/// Radar measurement noise standard deviation - bearing
static const double noise_bearing_    = 0.0009; // [rad]^2

/// Radar measurement noise standard deviation - range-rate
static const double noise_range_rate_ = 0.09;   // [m/s]^2

class MeasurementModelRadar : public MeasurementModel
{
public:
    /// \brief Constructor
    /// \param n_states dimension of the state vector
    explicit MeasurementModelRadar(const std::size_t n_states);
    virtual ~MeasurementModelRadar();

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
    virtual Eigen::MatrixXd getR() const;

private:
    const std::size_t n_observed_states_ = 3U;
};

#endif // MEASUREMENT_MODEL_RADAR_H
