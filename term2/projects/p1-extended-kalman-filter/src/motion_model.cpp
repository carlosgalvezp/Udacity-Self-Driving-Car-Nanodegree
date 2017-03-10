#include "motion_model.h"

MotionModel::MotionModel(std::size_t state_dimension):
    F_(Eigen::MatrixXd::Zero(state_dimension, state_dimension)),
    Q_(Eigen::MatrixXd::Zero(state_dimension, state_dimension))
{
}
