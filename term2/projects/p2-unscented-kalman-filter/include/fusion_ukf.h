#ifndef FUSION_UKF_H
#define FUSION_UKF_H

#include <Eigen/Dense>
#include "measurement_package.h"
#include "ukf.h"
#include "motion_model.h"
#include "measurement_model_lidar.h"
#include "measurement_model_radar.h"

static const std::size_t kNumberOfStates = 5U;

/// \brief Uses an UKF to fuse sensor data from lidar and radar to estimate
/// the position, orientation, speed and yaw rate of a target
class FusionUKF
{
public:
    FusionUKF();

    /// \brief Runs one cycle of the UKF using a given measurement
    /// \param meas_package the sensor data
    void processMeasurement(const MeasurementPackage& meas_package);

    /// \brief getState returns the current state
    /// \return the current state
    const Eigen::VectorXd& getState() const { return ukf_.getState(); }

    /// \brief getNISLidar the NIS from the lidar
    /// \return the NIS from the lidar
    double getNISLidar() const { return NIS_lidar_; }

    /// \brief getNISRadar the NIS from the radar
    /// \return the NIS from the radar
    double getNISRadar() const { return NIS_radar_; }

private:
    void initialize(const MeasurementPackage& measurement_pack);

    UKF ukf_;

    MotionModel motion_model_;

    MeasurementModelLidar sensor_model_lidar_;
    MeasurementModelRadar sensor_model_radar_;

    std::size_t previous_timestamp_;

    bool is_initialized_;

    // if false, laser measurements will be ignored (except for init)
    bool use_laser_;

    // if false, radar measurements will be ignored (except for init)
    bool use_radar_;

    double NIS_lidar_;
    double NIS_radar_;
};

#endif // FUSION_UKF_H
