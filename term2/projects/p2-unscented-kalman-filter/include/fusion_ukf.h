#ifndef FUSION_UKF_H
#define FUSION_UKF_H

#include <Eigen/Dense>
#include "measurement_package.h"
#include "ukf.h"
#include "motion_model.h"
#include "measurement_model_lidar.h"
#include "measurement_model_radar.h"

static const std::size_t kNumberOfStates = 5U;

class FusionUKF
{
public:
    FusionUKF();

    void processMeasurement(const MeasurementPackage& meas_package);
    const Eigen::VectorXd& getState() const { return ukf_.getState(); }
    double getNISLidar() const { return NIS_lidar_; }
    double getNISRadar() const { return NIS_radar_; }

private:
    void initialize(const MeasurementPackage& measurement_pack);

    /// Unscented Kalman Filter
    UKF ukf_;

    MotionModel motion_model_;

    MeasurementModelLidar sensor_model_lidar_;
    MeasurementModelRadar sensor_model_radar_;

    std::size_t previous_timestamp_;

    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    double NIS_lidar_;
    double NIS_radar_;
};

#endif // FUSION_UKF_H
