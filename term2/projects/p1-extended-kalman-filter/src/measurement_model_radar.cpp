#include "measurement_model_radar.h"

MeasurementModelRadar::MeasurementModelRadar():
    MeasurementModel()
{
    R_ = Eigen::MatrixXd(3, 3);
    H_ = Eigen::MatrixXd(3, 4);
}

MeasurementModelRadar::~MeasurementModelRadar()
{
}
