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

    /// \brief predictMeasurement computes z_hat = h(x')
    /// \param state predicted state, x'
    /// \return the predicted measurement, z_hat
    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const = 0;

    /// \brief getH computes and returns the H matrix
    /// \param state predicted state, x'
    /// \return the H matrix
    virtual Eigen::MatrixXd getH(const Eigen::VectorXd& state) const = 0;

    /// \brief getR computes and returns the R matrix
    /// \return the R matrix
    virtual Eigen::MatrixXd getR() const = 0;

protected:
    const std::size_t n_states_;
};

#endif // MEASUREMENTMODEL_H
