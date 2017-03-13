#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <Eigen/Dense>

class MeasurementModel
{
public:
    MeasurementModel(const std::size_t state_dimension);
    virtual ~MeasurementModel();

    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const = 0;

    virtual Eigen::MatrixXd getH(const Eigen::VectorXd& state) const = 0;
    virtual Eigen::MatrixXd getR() const = 0;

protected:
    const std::size_t n_states_;
};

#endif // MEASUREMENTMODEL_H
