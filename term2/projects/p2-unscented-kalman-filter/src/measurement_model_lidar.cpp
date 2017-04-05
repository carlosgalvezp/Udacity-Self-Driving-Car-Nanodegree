#include "measurement_model_lidar.h"

MeasurementModelLidar::MeasurementModelLidar(std::size_t n_states):
    MeasurementModel(n_states),
    R_(Eigen::MatrixXd::Zero(n_observed_states_, n_observed_states_))
{
    R_(0, 0) = std_px_ * std_px_;
    R_(1, 1) = std_py_ * std_py_;
}

MeasurementModelLidar::~MeasurementModelLidar()
{
}

Eigen::VectorXd MeasurementModelLidar::computeInitialState(
        const Eigen::VectorXd& z) const
{
    Eigen::VectorXd x(n_states_);
    x << z(0), z(1), 0.0, 0.0, 0.0;

    return x;
}

Eigen::VectorXd MeasurementModelLidar::predictMeasurement(
    const Eigen::VectorXd& state) const
{
    return state.head(n_observed_states_);
}

Eigen::VectorXd MeasurementModelLidar::computeDifference(
    const Eigen::VectorXd& z_a,
    const Eigen::VectorXd& z_b) const
{
    return z_a - z_b;
}
