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
    const Eigen::MatrixXd& H = getH(state);
    return H * state;
}

Eigen::VectorXd MeasurementModelLidar::computeResidual(const Eigen::VectorXd &z,
                                                       const Eigen::VectorXd &z_hat) const
{
    return z - z_hat;
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

    R << noise_px_, 0.0,
         0.0,      noise_py_;

    return R;
}
