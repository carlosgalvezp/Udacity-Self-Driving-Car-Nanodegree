#include "measurement_model.h"

MeasurementModel::MeasurementModel(const std::size_t state_dimension):
    n_states_(state_dimension)
{
}

MeasurementModel::~MeasurementModel()
{
}

