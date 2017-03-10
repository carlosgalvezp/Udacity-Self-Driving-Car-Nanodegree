#ifndef MEASUREMENT_MODEL_LIDAR_H
#define MEASUREMENT_MODEL_LIDAR_H

#include "measurement_model.h"

class MeasurementModelLidar : public MeasurementModel
{
public:
    MeasurementModelLidar(std::size_t state_dimension);
    virtual ~MeasurementModelLidar();

    const std::size_t n_observed_states = 2U;
};

#endif // MEASUREMENT_MODEL_LIDAR_H
