#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <Eigen/Dense>

/// \brief Interface that any sensor model must fulfill
class MeasurementModel
{
public:
    /// \brief Constructor
    /// \param n_states dimension of the state vector
    explicit MeasurementModel(const std::size_t n_states);
    virtual ~MeasurementModel();

    /// \brief computes the initialization state, given a measurement
    /// \param z initial measurement
    /// \return the initial state for the filter
    virtual Eigen::VectorXd computeInitialState(const Eigen::VectorXd& z) const = 0;

    /// \brief Computes the predicted measurement, z_hat = h(x')
    /// \param state predicted state, x'
    /// \return the predicted measurement, z_hat
    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const = 0;

    /// \brief Computes the difference between 2 measurements
    /// \param z_a first measurement
    /// \param z_b second measurement
    /// \return the difference y = z_a - z_b
    virtual Eigen::VectorXd computeDifference(const Eigen::VectorXd& z_a,
                                              const Eigen::VectorXd& z_b) const  = 0;


    /// \brief computes and returns the measurement noise matrix, R
    /// \return the R matrix
    virtual Eigen::MatrixXd getR() const = 0;

protected:
    const std::size_t n_states_;
};

#endif // MEASUREMENTMODEL_H
