#include "measurement_model_lidar.h"

MeasurementModelLidar::MeasurementModelLidar(std::size_t n_states):
    MeasurementModel(n_states)
{
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;
}

MeasurementModelLidar::~MeasurementModelLidar()
{
}

Eigen::VectorXd MeasurementModelLidar::predictMeasurement(const Eigen::VectorXd& state) const
{
    Eigen::VectorXd z_out(n_observed_states_);
    z_out << state[0], state[1];
    return z_out;
}

Eigen::VectorXd MeasurementModelLidar::computeDifference(const Eigen::VectorXd &z_a,
                                                       const Eigen::VectorXd &z_b) const
{
    return z_a - z_b;
}

Eigen::MatrixXd MeasurementModelLidar::getR() const
{
    Eigen::MatrixXd R(n_observed_states_, n_observed_states_);

    R << noise_px_, 0.0,
         0.0,      noise_py_;

    return R;
}
