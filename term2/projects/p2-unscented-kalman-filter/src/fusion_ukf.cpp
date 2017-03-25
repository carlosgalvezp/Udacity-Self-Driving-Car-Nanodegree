#include "fusion_ukf.h"
#include "constants.h"

FusionUKF::FusionUKF():
    ukf_(kNumberOfStates),
    motion_model_(),
    sensor_model_lidar_(kNumberOfStates),
    sensor_model_radar_(kNumberOfStates)
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

}

void FusionUKF::initialize(const MeasurementPackage& meas_package)
{
    (void) meas_package;
}

void FusionUKF::processMeasurement(const MeasurementPackage& meas_package)
{
    // Initialize
    if (!initialized_)
    {
        initialize(meas_package);
        return;
    }

    // Predict
    const std::size_t new_timestamp = meas_package.timestamp_;
    const double delta_t = (new_timestamp - current_timestamp_) * kMicroSecToSec;
    ukf_.predict(motion_model_, delta_t);

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
