#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Dense>

class MotionModel
{
public:
    MotionModel(std::size_t state_dimension);

    Eigen::VectorXd predict(const Eigen::VectorXd& state,
                            const double delta_t) const;

    Eigen::MatrixXd getTransitionMatrix(const double delta_t) const;
    Eigen::MatrixXd getProcessNoise(const double delta_t) const;

private:
    const std::size_t state_dimension_;

    // Process noise
    // TODO: tweak. Should be variance, not std
    const double sigma_ax_ = 10.0;  // [m]
    const double sigma_ay_ = 10.0;  // [m]
};

#endif // MOTION_MODEL_H
