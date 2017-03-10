#include "measurement_model_lidar.h"

MeasurementModelLidar::MeasurementModelLidar(std::size_t state_dimension):
    MeasurementModel()
{
    R_ = Eigen::MatrixXd(n_observed_states, n_observed_states);
    H_ = Eigen::MatrixXd(n_observed_states, state_dimension);
}

MeasurementModelLidar::~MeasurementModelLidar()
{
}
