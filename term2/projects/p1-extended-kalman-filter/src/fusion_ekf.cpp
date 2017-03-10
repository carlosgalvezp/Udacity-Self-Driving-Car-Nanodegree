#include "fusion_ekf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

FusionEKF::FusionEKF():
    is_initialized_(false),
    previous_timestamp_(0),
    ekf_(state_dimension),
    motion_model_(state_dimension),
    meas_model_lidar_(state_dimension),
    meas_model_radar_(state_dimension)
{
}

void FusionEKF::processMeasurement(const MeasurementPackage& measurement_pack)
{
    // Prediction
    const long new_timestamp = measurement_pack.timestamp_;
    const double delta_t = static_cast<double>(new_timestamp - previous_timestamp_) * 1.0E-6;

    ekf_.predict(motion_model_, delta_t);

    previous_timestamp_ = new_timestamp;

    // Update
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        ekf_.update(meas_model_radar_, measurement_pack.raw_measurements_);
    }
    else
    {
        // Laser updates
        ekf_.update(meas_model_lidar_, measurement_pack.raw_measurements_);
    }

    // print the output
    std::cout << "x_ = " << ekf_.getState() << std::endl;
    std::cout << "P_ = " << ekf_.getCovariance() << std::endl;
}
