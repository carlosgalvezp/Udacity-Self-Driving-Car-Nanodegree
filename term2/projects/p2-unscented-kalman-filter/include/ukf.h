#ifndef UKF_H
#define UKF_H
#include <Eigen/Dense>
#include <vector>

#include "measurement_package.h"
#include "motion_model.h"
#include "measurement_model.h"

class UKF
{
public:
    explicit UKF(const std::size_t n_states);

    const Eigen::VectorXd& getState() const { return x_; }

    void generateSigmaPoints();
    void predict(const MotionModel& motion_model, const double delta_t);
    void update(const MeasurementModel& sensor_model, const Eigen::VectorXd& z);
private:

    /// Dimension of the state vector
    const std::size_t n_states_;

    /// State vector: [pos_x pos_y velocity yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    /// State covariance matrix
    Eigen::MatrixXd P_;

    /// Predicted sigma points
    std::vector<Eigen::VectorXd> x_sig_pred_;

    /// Weights of sigma points
    std::vector<double> weights_;
};

#endif /* UKF_H */
