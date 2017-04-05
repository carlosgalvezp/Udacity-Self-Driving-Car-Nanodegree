#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Dense>

/// Process noise standard deviation longitudinal acceleration
static const double std_a_     = 0.8;   // [m/s^2]

/// Process noise standard deviation rotational acceleration
static const double std_yawdd_ = 0.6;  // [rad/s^2]

/// \brief Implements the motion model
class MotionModel
{
public:
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

    /// \brief Returns the size of the state vector
    /// \return the size of the state vector
    std::size_t getStateVectorSize() const { return kNumberOfStates; }

    /// \brief Returns the size of the augmented vector
    /// \return the size of the augmented vector
    std::size_t getAugStateVectorSize() const { return kNumberOfStatesAugmented; }

private:
    // Size of the state vector
    static const std::size_t kNumberOfStates = 5U;

    // Size of the augmented vector
    static const std::size_t kNumberOfStatesAugmented = 7U;

    // Number of independent noise sources in the motion model
    static const std::size_t kNoiseVectorSize = 2U;

    // Process noise matrix
    Eigen::MatrixXd Q_;
};

#endif // MOTION_MODEL_H
