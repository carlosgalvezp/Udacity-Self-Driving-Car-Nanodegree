#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <Eigen/Dense>

class MeasurementModel
{
public:
    /// \brief Constructor
    /// \param n_states dimension of the state vector
    MeasurementModel(const std::size_t n_states);
    virtual ~MeasurementModel();

    /// \brief Computes the predicted measurement, z_hat = h(x')
    /// \param state predicted state, x'
    /// \return the predicted measurement, z_hat
    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const = 0;

    /// \brief Computes the residual, y = z - z_hat
    /// \param z sensor measurement
    /// \param z_hat predicted measurement
    /// \return the residual, y
    virtual Eigen::VectorXd computeResidual(const Eigen::VectorXd& z,
                                            const Eigen::VectorXd& z_hat) const  = 0;

    /// \brief Computes and returns the measurement matrix, H
    /// \param state predicted state, x'
    /// \return the H matrix
    virtual Eigen::MatrixXd getH(const Eigen::VectorXd& state) const = 0;

    /// \brief computes and returns the measurement noise matrix, R
    /// \return the R matrix
    virtual Eigen::MatrixXd getR() const = 0;

protected:
    const std::size_t n_states_;
};

#endif // MEASUREMENTMODEL_H
