#include "measurement_model_lidar.h"

MeasurementModelLidar::MeasurementModelLidar():
    MeasurementModel()
{
    R_ = Eigen::MatrixXd(2, 2);
    H_ = Eigen::MatrixXd(2, 4);
}

MeasurementModelLidar::~MeasurementModelLidar()
{
}
