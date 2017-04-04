#include "measurement_model_radar.h"
#include "tools.h"
#include <cmath>

MeasurementModelRadar::MeasurementModelRadar(const std::size_t n_states):
    MeasurementModel(n_states)
{
    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.0175;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.1;
}

MeasurementModelRadar::~MeasurementModelRadar()
{
}

Eigen::VectorXd MeasurementModelRadar::predictMeasurement(const Eigen::VectorXd& state) const
{
    Eigen::VectorXd z_hat = Eigen::VectorXd::Zero(n_observed_states_);

    const double px = state(0);
    const double py = state(1);
    const double v = state(2);
    const double yaw = state(3);

    const double vx = v * std::cos(yaw);
    const double vy = v * std::sin(yaw);

    const double sqrt_sum = std::sqrt(px*px + py*py);

    if (Tools::isNotZero(sqrt_sum))
    {
        z_hat << sqrt_sum,
                 std::atan2(py, px),
                 (px * vx + py * vy) / sqrt_sum;
    }

    return z_hat;
}

Eigen::VectorXd MeasurementModelRadar::computeDifference(const Eigen::VectorXd &z_a,
                                                         const Eigen::VectorXd &z_b) const
{
    Eigen::VectorXd y = z_a - z_b;
    y(1) = Tools::normalizeAngle(y(1));

    return y;
}

Eigen::MatrixXd MeasurementModelRadar::getR() const
{
    Eigen::MatrixXd R(n_observed_states_, n_observed_states_);

    R << noise_range_, 0.0,            0.0,
         0.0,          noise_bearing_, 0.0,
         0.0,          0.0,            noise_range_rate_;

    return R;
}
