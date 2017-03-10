#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <Eigen/Dense>

class MotionModel
{
public:
    MotionModel();

    const Eigen::MatrixXd& getTransitionMatrix() const { return F_; }
    const Eigen::MatrixXd& getProcessNoise() const { return Q_; }

private:
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
};

#endif // MOTION_MODEL_H
