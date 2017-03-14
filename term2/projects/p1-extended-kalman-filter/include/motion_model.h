#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Dense>


// Process noise
// TODO: tweak. Should be variance, not std
static const double sigma_ax_ = 9.0;  // [m/s^2]^2
static const double sigma_ay_ = 9.0;  // [m/s^2]^2

class MotionModel
{
public:
    MotionModel(std::size_t state_dimension);

    Eigen::VectorXd predict(const Eigen::VectorXd& state,
                            const double delta_t) const;

    Eigen::MatrixXd getF(const double delta_t) const;
    Eigen::MatrixXd getQ(const double delta_t) const;

private:
    const std::size_t state_dimension_;

};

#endif // MOTION_MODEL_H
