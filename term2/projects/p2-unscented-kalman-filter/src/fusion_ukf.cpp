#include "fusion_ukf.h"

FusionUKF::FusionUKF():
    ukf_(kNumberOfStates),
    sensor_model_lidar_(kNumberOfStates),
    sensor_model_radar_(kNumberOfStates)
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

}

void FusionUKF::processMeasurement(const MeasurementPackage& /*measurement*/)
{
}
