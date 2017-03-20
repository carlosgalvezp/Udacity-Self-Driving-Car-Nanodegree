#ifndef MEASUREMENT_MODEL_LIDAR_H
#define MEASUREMENT_MODEL_LIDAR_H

#include "measurement_model.h"

// Measurement noise
static const double noise_px_ = 0.0225;  // [m]^2
static const double noise_py_ = 0.0225;  // [m]^2

class MeasurementModelLidar : public MeasurementModel
{
public:
    /// \brief Constructor
    /// \param n_states dimension of the state vector
    MeasurementModelLidar(std::size_t n_states);
    virtual ~MeasurementModelLidar();

    /// \brief Computes z_hat = h(x')
    /// \param state predicted state, x'
    /// \return the predicted measurement, z_hat
    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const;

    /// \brief Computes the residual, y = z - z_hat
    /// \param z sensor measurement
    /// \param z_hat predicted measurement
    /// \return the residual, y
    virtual Eigen::VectorXd computeResidual(const Eigen::VectorXd& z,
                                            const Eigen::VectorXd& z_hat) const;

    /// \brief Computes and returns the measurement matrix, H
    /// \param state predicted state, x'
    /// \return the H matrix
    virtual Eigen::MatrixXd getH(const Eigen::VectorXd &state) const;

    /// \brief computes and returns the measurement noise matrix, R
    /// \return the R matrix
    virtual Eigen::MatrixXd getR() const;

private:
    const std::size_t n_observed_states = 2U;
};

#endif // MEASUREMENT_MODEL_LIDAR_H
