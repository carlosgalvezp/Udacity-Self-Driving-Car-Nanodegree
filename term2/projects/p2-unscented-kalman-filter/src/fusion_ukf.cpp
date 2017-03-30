#include "fusion_ukf.h"
#include "constants.h"
#include "tools.h"

FusionUKF::FusionUKF():
    ukf_(kNumberOfStates, motion_model_),
    motion_model_(),
    sensor_model_lidar_(kNumberOfStates),
    sensor_model_radar_(kNumberOfStates)
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

}

void FusionUKF::initialize(const MeasurementPackage& measurement_pack)
{
    double px = 0.0;
    double py = 0.0;

    if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        const double rho = measurement_pack.raw_measurements_[0];
        const double phi = measurement_pack.raw_measurements_[1];

        px = rho * std::cos(phi);
        py = rho * std::sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        px = measurement_pack.raw_measurements_[0];
        py = measurement_pack.raw_measurements_[1];
    }

    Eigen::VectorXd x0(kNumberOfStates);
    x0 << px, py, 0.0, 0.0, 0.0;

    if (Tools::isNotZero(x0.norm()))
    {
        ukf_.setState(x0);
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
    }
}

void FusionUKF::processMeasurement(const MeasurementPackage& meas_package)
{
    // Initialize
    if (!is_initialized_)
    {
        initialize(meas_package);
        return;
    }

    // Predict
    const std::size_t new_timestamp = meas_package.timestamp_;
    const double delta_t = (new_timestamp - previous_timestamp_) * kMicroSecToSec;

    ukf_.predict(motion_model_, delta_t);

    previous_timestamp_ = new_timestamp;

    // Update
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {
        ukf_.update(sensor_model_lidar_, meas_package.raw_measurements_);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
        ukf_.update(sensor_model_radar_, meas_package.raw_measurements_);
    }
}
