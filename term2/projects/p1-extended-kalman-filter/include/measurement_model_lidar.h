#ifndef MEASUREMENT_MODEL_LIDAR_H
#define MEASUREMENT_MODEL_LIDAR_H

#include "measurement_model.h"

// Measurement noise
// TODO: tweak. should be variance, not std
static const double sigma_px_ = 0.0225;  // [m]^2
static const double sigma_py_ = 0.0225;  // [m]^2

class MeasurementModelLidar : public MeasurementModel
{
public:
    MeasurementModelLidar(std::size_t state_dimension);
    virtual ~MeasurementModelLidar();

    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const;

    virtual Eigen::MatrixXd getH(const Eigen::VectorXd &state) const;
    virtual Eigen::MatrixXd getR() const;

private:
    const std::size_t n_observed_states = 2U;
};

#endif // MEASUREMENT_MODEL_LIDAR_H
