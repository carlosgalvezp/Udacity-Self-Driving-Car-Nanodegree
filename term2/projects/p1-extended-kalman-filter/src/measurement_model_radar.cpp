#include "measurement_model_radar.h"
#include <cmath>

MeasurementModelRadar::MeasurementModelRadar(const std::size_t state_dimension):
    MeasurementModel(state_dimension)
{
}

MeasurementModelRadar::~MeasurementModelRadar()
{
}

Eigen::VectorXd MeasurementModelRadar::predictMeasurement(const Eigen::VectorXd& state) const
{
    Eigen::VectorXd z_hat(n_observed_states_);

    const double px = state(0);
    const double py = state(1);
    const double vx = state(2);
    const double vy = state(3);

    const double sqrt_sum = std::sqrt(px*px + py*py);

    z_hat << sqrt_sum,
             std::atan2(py, px),
             (px * vx + py * vy) / sqrt_sum;

    return z_hat;
}

Eigen::MatrixXd MeasurementModelRadar::getMeasurementMatrix(const Eigen::VectorXd &state) const
{
    const double px = state(0);
    const double py = state(1);
    const double vx = state(2);
    const double vy = state(3);

    const double sum_sq = px*px + py*py;
    const double sqrt_sum = std::sqrt(sum_sq);
    const double sum_3_2 = sum_sq * sqrt_sum;

    Eigen::MatrixXd H(n_observed_states_, n_states_);

    H << px / sqrt_sum,                  py / sqrt_sum,                  0.0,           0.0,
        -py / sum_sq,                    px / sum_sq,                    0.0,           0.0,
         py * (vx*py - vy*px) / sum_3_2, px * (vy*px - vx*py) / sum_3_2, px / sqrt_sum, py / sqrt_sum;

    return H;
}

Eigen::MatrixXd MeasurementModelRadar::getMeasurementNoise() const
{
    Eigen::MatrixXd R(n_observed_states_, n_observed_states_);

    R << sigma_range_, 0.0,            0.0,
         0.0,          sigma_bearing_, 0.0,
         0.0,          0.0,            sigma_range_rate_;

    return R;
}
