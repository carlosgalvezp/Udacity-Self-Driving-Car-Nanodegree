#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
                        Eigen::MatrixXd& F_in, Eigen::MatrixXd& H_in,
                        Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
    I_ = Eigen::MatrixXd::Identity(x_.cols(), x_.cols());
}

void KalmanFilter::predict()
{
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z)
{
    Eigen::VectorXd y = z - H_ * x_;

    Eigen::MatrixXd Ht = H_.transpose();

    Eigen::MatrixXd S = H_ * P_ * Ht * R_;
    Eigen::MatrixXd K = P_ * Ht * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;
}

void KalmanFilter::updateEKF(const Eigen::VectorXd& z)
{
    Eigen::VectorXd y = z - Hj_ * x_;

    Eigen::MatrixXd Ht = Hj_.transpose();

    Eigen::MatrixXd S = H_ * P_ * Ht * R_;
    Eigen::MatrixXd K = P_ * Ht * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * Hj_) * P_;
}
