#ifndef MEASUREMENT_MODEL_LIDAR_H
#define MEASUREMENT_MODEL_LIDAR_H

#include "measurement_model.h"

class MeasurementModelLidar : public MeasurementModel
{
public:
    MeasurementModelLidar(std::size_t state_dimension);
    virtual ~MeasurementModelLidar();

    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const;

    virtual Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd &state) const;
    virtual Eigen::MatrixXd getMeasurementNoise() const;

private:
    const std::size_t n_observed_states = 2U;

    const double sigma_px_ = 0.2;
    const double sigma_py_ = 0.2;
};

#endif // MEASUREMENT_MODEL_LIDAR_H
