#ifndef FUSION_UKF_H
#define FUSION_UKF_H

#include <Eigen/Dense>
#include "measurement_package.h"
#include "ukf.h"
#include "measurement_model_lidar.h"
#include "measurement_model_radar.h"

class FusionUKF
{
public:
    FusionUKF();

    void processMeasurement(const MeasurementPackage& measurement);
    const Eigen::VectorXd& getState() const { return ukf_.getState(); }

private:
    /// Unscented Kalman Filter
    UKF ukf_;

    MeasurementModelLidar sensor_model_lidar_;
    MeasurementModelRadar sensor_model_radar_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

};

#endif // FUSION_UKF_H
