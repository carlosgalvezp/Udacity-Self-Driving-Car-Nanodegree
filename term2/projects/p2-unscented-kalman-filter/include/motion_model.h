#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Dense>

/// Process noise standard deviation longitudinal acceleration
static const double std_a_     = 3.0;   // [m/s^2]^2

// Process noise standard deviation rotational acceleration
static const double std_yawdd_ = 0.03;  // [rad/s^2]^2

/// Number of independent noise sources in the motion model
static const std::size_t kNoiseVectorSize = 2U;

/// Size of the augmented vector
static const std::size_t kAugmentedStateSize = 7U;

class MotionModel
{
public:    
    /// \brief Constructor
    MotionModel();

    /// \brief Computes x' = f(x)
    /// \param state current state, x
    /// \param delta_t time difference w.r.t the previous prediction step
    /// \return the predicted state, x'
    Eigen::VectorXd predict(const Eigen::VectorXd& x,
                            const double delta_t) const;

    /// \brief Returns the process noise matrix, Q
    /// \return the Q matrix
    const Eigen::MatrixXd& getQ() const { return Q_; }

    std::size_t getAugmentedSize() const { return kAugmentedStateSize; }

private:
    Eigen::MatrixXd Q_;
};

#endif // MOTION_MODEL_H
