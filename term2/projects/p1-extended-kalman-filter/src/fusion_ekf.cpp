#include "fusion_ekf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

/*
 * Constructor.
 */
FusionEKF::FusionEKF():
    is_initialized_(false),
    previous_timestamp_(0ULL),
    ekf_(),
    motion_model_(),
    meas_model_lidar_(),
    meas_model_radar_()
{
    /**
    TODO:
      * Finish initializing the FusionEKF.
    */
}

void FusionEKF::processMeasurement(const MeasurementPackage& measurement_pack)
{
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
     */

    const double delta_t = 0.;
    ekf_.predict(motion_model_, delta_t);

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
    }
    else
    {
        // Laser updates
    }

    // print the output
    std::cout << "x_ = " << ekf_.getState() << std::endl;
    std::cout << "P_ = " << ekf_.getCovariance() << std::endl;
}
