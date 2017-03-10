#ifndef MEASUREMENT_MODEL_RADAR_H
#define MEASUREMENT_MODEL_RADAR_H

#include "measurement_model.h"

// TODO: tweak. should be variance, not std
static const double sigma_range_      = 0.2;                       // [m]
static const double sigma_bearing_    = (10.0 * M_PI / 180.0);     // [rad]
static const double sigma_range_rate_ = 1.0;                       // [m/s]

class MeasurementModelRadar : public MeasurementModel
{
public:
    MeasurementModelRadar(const std::size_t state_dimension);
    virtual ~MeasurementModelRadar();

    virtual Eigen::VectorXd predictMeasurement(const Eigen::VectorXd& state) const;

    virtual Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd &state) const;
    virtual Eigen::MatrixXd getMeasurementNoise() const;

private:
    const std::size_t n_observed_states_ = 3U;
};

#endif // MEASUREMENT_MODEL_RADAR_H
