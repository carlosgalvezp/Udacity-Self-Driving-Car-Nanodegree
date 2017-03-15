#include "kalman_filter.h"
#include "tools.h"

KalmanFilter::KalmanFilter(const std::size_t n_states):
    x_(Eigen::VectorXd::Constant(n_states, x0_)),
    P_(Eigen::MatrixXd::Constant(n_states, n_states, p0_)),
    I_(Eigen::MatrixXd::Identity(n_states, n_states))
{
}

void KalmanFilter::predict(const MotionModel& motion_model,
                           const double delta_t)
{
    const Eigen::MatrixXd F = motion_model.getF(delta_t);
    const Eigen::MatrixXd Q = motion_model.getQ(delta_t);

    x_ = motion_model.predict(x_, delta_t);
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const MeasurementModel& sensor_model,
                          const Eigen::VectorXd& z)
{
    const Eigen::MatrixXd H = sensor_model.getH(x_);
    const Eigen::MatrixXd R = sensor_model.getR();

    const Eigen::VectorXd z_hat = sensor_model.predictMeasurement(x_);

    const Eigen::VectorXd y = z - z_hat;

    const Eigen::MatrixXd Ht = H.transpose();

    const Eigen::MatrixXd S = H * P_ * Ht + R;

    if(Tools::isNotZero(S.determinant()))
    {
        const Eigen::MatrixXd K = P_ * Ht * S.inverse();

        x_ = x_ + K * y;
        P_ = (I_ - K * H) * P_;
    }
}
