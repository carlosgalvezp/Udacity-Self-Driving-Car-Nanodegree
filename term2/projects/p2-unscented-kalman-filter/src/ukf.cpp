#include <iostream>
#include "ukf.h"
#include "tools.h"

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(const std::size_t n_states, const MotionModel &motion_model)
    : n_states_(n_states),
      n_augmented_(motion_model_.getAugmentedSize()),
      x_new_(),
      x_(Eigen::VectorXd::Zero(n_states)),
      P_(Eigen::MatrixXd::Zero(n_states, n_states)),
      motion_model_(motion_model),
      x_sig_pred_(),
      weights_(computeNumberOfSigmaPoints(n_augmented_))
{
    // Initial covariance matrix
    const double p0 = 10.0;
    for (int i = 0U; i < P_.rows(); ++i)
    {
        P_(i,i) = p0;
    }

    // Precompute weights
    // TODO: make this nicer
    const double lambda = 3.0 - n_augmented_;
    weights_[0] = lambda / (lambda + n_augmented_);

    for(std::size_t i = 1U; i < weights_.size(); ++i)
    {
        weights_[i] = 0.5 / (lambda + n_augmented_);
    }
}

std::size_t UKF::computeNumberOfSigmaPoints(const std::size_t n_states) const
{
    return 2U * n_states + 1U;
}

void UKF::generateSigmaPoints(const Eigen::VectorXd& x,
                              const Eigen::MatrixXd& P,
                              std::vector<Eigen::VectorXd>& x_sig)
{
    const std::size_t n_states = x.rows();
    const std::size_t n_sigma_pts = computeNumberOfSigmaPoints(n_states);
    const double lambda = 3.0 - n_states;

    const Eigen::MatrixXd P_sqrt = Tools::sqrt(P);

    x_sig.resize(n_sigma_pts);

    x_sig[0] = x;

    for(std::size_t i = 0U; i < n_states; ++i)
    {
        x_sig[1U + i]             = x + (lambda + n_states) * P_sqrt.col(i);
        x_sig[1U + n_states + i]  = x - (lambda + n_states) * P_sqrt.col(i);
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::predict(const MotionModel& motion_model, const double delta_t)
{
    const Eigen::MatrixXd Q = motion_model.getQ();
    const std::size_t n_augmented = motion_model.getAugmentedSize();

    Eigen::VectorXd x_a = Eigen::VectorXd::Zero(n_augmented);
    Eigen::MatrixXd P_a = Eigen::MatrixXd::Zero(n_augmented, n_augmented);

    x_a.head(n_states_) = x_;
    P_a.topLeftCorner(n_states_, n_states_) = P_;
    P_a.bottomRightCorner(Q.rows(), Q.cols()) = Q;

    // Generate sigma points
    generateSigmaPoints(x_a, P_a, x_sig_pred_);

    // Predict them
    for (std::size_t i = 0U; i < x_sig_pred_.size(); ++i)
    {
        x_sig_pred_[i] = motion_model.predict(x_sig_pred_[i], delta_t);
    }

    // Compute predicted mean
    x_.fill(0.0);
    for (std::size_t i = 0U; i < weights_.size(); ++i)
    {
        x_ += weights_[i] * x_sig_pred_[i].head(n_states_);
    }

    // Compute predicted covariance
    P_.fill(0.0);
    for (std::size_t i = 0U; i < weights_.size(); ++i)
    {
        const Eigen::VectorXd x_diff = x_sig_pred_[i].head(n_states_) - x_;
        Tools::normalizeAngle(x_diff[3]);

        P_ += weights_[i] * x_diff * x_diff.transpose();
    }
}

double UKF::update(const MeasurementModel& sensor_model, const Eigen::VectorXd& z)
{
    const std::size_t n_observed = z.rows();
    double nis = 0.0;

    // Predict the measurement for each sigma point
    std::vector<Eigen::VectorXd> z_sig_pred(x_sig_pred_.size());

    for(std::size_t i = 0U; i < z_sig_pred.size(); ++i)
    {
        z_sig_pred[i] = sensor_model.predictMeasurement(x_sig_pred_[i]);
    }

    // Compute mean
    Eigen::VectorXd z_mean = Eigen::VectorXd::Zero(n_observed);
    for(std::size_t i = 0U; i < z_sig_pred.size(); ++i)
    {
        z_mean += weights_[i] * z_sig_pred[i];
    }

    // Compute covariance and cross-correlation
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n_observed, n_observed);
    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(n_states_, n_observed);
    for(std::size_t i = 0U; i < z_sig_pred.size(); ++i)
    {
        Eigen::VectorXd z_diff = sensor_model.computeDifference(z_sig_pred[i], z_mean);

        S += weights_[i] * z_diff * z_diff.transpose();

        Eigen::VectorXd x_diff = x_sig_pred_[i].head(n_states_) - x_;
        x_diff[3] = Tools::normalizeAngle(x_diff[3]);

        T += weights_[i] * x_diff * z_diff.transpose();
    }

    S += sensor_model.getR();

    // Perform update step
    if (Tools::isNotZero(S.determinant()))
    {
        Eigen::MatrixXd K = T * S.inverse();

        Eigen::VectorXd z_diff = sensor_model.computeDifference(z, z_mean);

        x_ = x_ + K * z_diff;
        P_ = P_ - K * S * K.transpose();

        // Compute NIS
        nis = z_diff.transpose() * S.inverse() * z_diff;
    }

    return nis;
}
