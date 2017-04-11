#include "fusion_ukf.h"
#include "constants.h"
#include "tools.h"

FusionUKF::FusionUKF():
    ukf_(motion_model_),
    motion_model_(),
    sensor_model_lidar_(motion_model_.getStateVectorSize()),
    sensor_model_radar_(motion_model_.getStateVectorSize()),
    previous_timestamp_(0U),
    is_initialized_(false),
    use_laser_(true),
    use_radar_(true),
    NIS_lidar_(0.0),
    NIS_radar_(0.0)
{
}

void FusionUKF::initialize(const MeasurementPackage& measurement_pack)
{
    Eigen::VectorXd x0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        x0 = sensor_model_radar_.computeInitialState(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
        x0 = sensor_model_lidar_.computeInitialState(measurement_pack.raw_measurements_);
    }

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
        NIS_lidar_ = ukf_.update(sensor_model_lidar_, meas_package.raw_measurements_);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
        NIS_radar_ = ukf_.update(sensor_model_radar_, meas_package.raw_measurements_);
    }
}
