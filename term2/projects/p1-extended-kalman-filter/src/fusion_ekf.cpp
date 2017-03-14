#include "fusion_ekf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

FusionEKF::FusionEKF():
    is_initialized_(false),
    previous_timestamp_(0),
    ekf_(n_states_),
    motion_model_(n_states_),
    meas_model_lidar_(n_states_),
    meas_model_radar_(n_states_)
{
}

void FusionEKF::initialize(const MeasurementPackage& measurement_pack)
{
    double px = 0.0;
    double py = 0.0;
    double vx = 0.0;
    double vy = 0.0;

    if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        const double rho = measurement_pack.raw_measurements_[0];
        const double phi = measurement_pack.raw_measurements_[1];
        const double rho_dot = measurement_pack.raw_measurements_[2];

        px = rho * std::cos(phi);
        py = rho * std::sin(phi);

        vx = rho_dot * std::cos(phi);
        vy = rho_dot * std::sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        px = measurement_pack.raw_measurements_[0];
        py = measurement_pack.raw_measurements_[1];
    }

    Eigen::VectorXd x0(n_states_);
    x0 << px, py, vx, vy;
    ekf_.setState(x0);

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
}

void FusionEKF::processMeasurement(const MeasurementPackage& measurement_pack)
{
    if(!is_initialized_)
    {
        initialize(measurement_pack);
        return;
    }

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
    else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        // Laser updates
        ekf_.update(meas_model_lidar_, measurement_pack.raw_measurements_);
    }

    // print the output
    std::cout << "x_ = " << ekf_.getState() << std::endl;
    std::cout << "P_ = " << ekf_.getCovariance() << std::endl;
}
