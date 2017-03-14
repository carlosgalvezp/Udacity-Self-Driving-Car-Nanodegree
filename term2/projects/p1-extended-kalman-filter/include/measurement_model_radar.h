#ifndef MEASUREMENT_MODEL_RADAR_H
#define MEASUREMENT_MODEL_RADAR_H

#include "measurement_model.h"

// TODO: tweak. should be variance, not std
static const double sigma_range_      = 0.09;   // [m]^2
static const double sigma_bearing_    = 0.0009; // [rad]^2
static const double sigma_range_rate_ = 0.09;   // [m/s]^2

class MeasurementModelRadar : public MeasurementModel
{
public:
    MeasurementModelRadar(const std::size_t state_dimension);
    virtual ~MeasurementModelRadar();

    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const;

    virtual Eigen::MatrixXd getH(const Eigen::VectorXd &state) const;
    virtual Eigen::MatrixXd getR() const;

private:
    const std::size_t n_observed_states_ = 3U;
};

#endif // MEASUREMENT_MODEL_RADAR_H
