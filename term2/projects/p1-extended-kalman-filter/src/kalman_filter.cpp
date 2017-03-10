#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {}

void KalmanFilter::predict(const MotionModel& motion_model, const double /*delta_t*/)
{
    const Eigen::MatrixXd& F = motion_model.getTransitionMatrix();
    const Eigen::MatrixXd& Q = motion_model.getProcessNoise();

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const MeasurementModel& sensor_model,
                          const Eigen::VectorXd& z)
{
    const Eigen::MatrixXd& H = sensor_model.getMeasurementMatrix();
    const Eigen::MatrixXd& R = sensor_model.getMeasurementNoise();

    Eigen::VectorXd y = z - H * x_;

    Eigen::MatrixXd Ht = H.transpose();

    Eigen::MatrixXd S = H * P_ * Ht * R;
    Eigen::MatrixXd K = P_ * Ht * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H) * P_;
}
