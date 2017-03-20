#include "kalman_filter.h"
#include "tools.h"

KalmanFilter::KalmanFilter(const std::size_t n_states):
    x_(Eigen::VectorXd::Zero(n_states)),
    P_(Eigen::MatrixXd::Zero(n_states, n_states)),
    I_(Eigen::MatrixXd::Identity(n_states, n_states))
{
    P_ << p0p_, 0.0,   0.0,  0.0,
          0.0,  p0p_,  0.0,  0.0,
          0.0,  0.0,   p0v_, 0.0,
          0.0,  0.0,   0.0,  p0v_;
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

    const Eigen::MatrixXd Ht = H.transpose();
    const Eigen::MatrixXd S = H * P_ * Ht + R;

    if(Tools::isNotZero(S.determinant()))
    {
        const Eigen::MatrixXd K = P_ * Ht * S.inverse();
        const Eigen::VectorXd z_hat = sensor_model.predictMeasurement(x_);
        const Eigen::VectorXd y = sensor_model.computeResidual(z, z_hat);

        x_ = x_ + K * y;
        P_ = (I_ - K * H) * P_;
    }
}
