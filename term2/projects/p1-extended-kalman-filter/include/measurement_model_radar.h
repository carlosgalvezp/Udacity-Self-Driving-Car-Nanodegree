#ifndef MEASUREMENT_MODEL_RADAR_H
#define MEASUREMENT_MODEL_RADAR_H

#include "measurement_model.h"

class MeasurementModelRadar : public MeasurementModel
{
public:
    MeasurementModelRadar(std::size_t state_dimension);
    virtual ~MeasurementModelRadar();

    const std::size_t n_observed_states = 3U;
};

#endif // MEASUREMENT_MODEL_RADAR_H
