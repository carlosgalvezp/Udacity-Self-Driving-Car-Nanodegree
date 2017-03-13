#include "measurement_model_lidar.h"

MeasurementModelLidar::MeasurementModelLidar(std::size_t state_dimension):
    MeasurementModel(state_dimension)
{
}

MeasurementModelLidar::~MeasurementModelLidar()
{
}

Eigen::VectorXd MeasurementModelLidar::predictMeasurement(const Eigen::VectorXd& state) const
{
    const Eigen::MatrixXd& H = getH(state);
    return H * state;
}

Eigen::MatrixXd MeasurementModelLidar::getH(const Eigen::VectorXd &state) const
{
    (void) state;  // Not required for this sensor model

    Eigen::MatrixXd H(n_observed_states, n_states_);

    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    return H;
}

Eigen::MatrixXd MeasurementModelLidar::getR() const
{
    Eigen::MatrixXd R(n_observed_states, n_observed_states);

    R << sigma_px_, 0.0,
         0.0,      sigma_py_;

    return R;
}
