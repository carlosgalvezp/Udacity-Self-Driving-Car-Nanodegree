#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Dense>


// Process noise
static const double noise_ax_ = 9.0;  // [m/s^2]^2
static const double noise_ay_ = 9.0;  // [m/s^2]^2

class MotionModel
{
public:    
    /// \brief Constructor
    /// \param n_states dimension of the state vector
    MotionModel(std::size_t n_states);

    /// \brief predict computes x' = f(x)
    /// \param state current state, x
    /// \param delta_t time difference w.r.t the previous prediction step
    /// \return the predicted state, x'
    Eigen::VectorXd predict(const Eigen::VectorXd& state,
                            const double delta_t) const;

    /// \brief getF computes and returns the F matrix
    /// \param delta_t time difference w.r.t the previous prediction step
    /// \return the F matrix
    Eigen::MatrixXd getF(const double delta_t) const;

    /// \brief getQ computes and returns the Q matrix
    /// \param delta_t time difference w.r.t the previous prediction step
    /// \return the Q matrix
    Eigen::MatrixXd getQ(const double delta_t) const;

private:
    const std::size_t n_states_;
};

#endif // MOTION_MODEL_H
