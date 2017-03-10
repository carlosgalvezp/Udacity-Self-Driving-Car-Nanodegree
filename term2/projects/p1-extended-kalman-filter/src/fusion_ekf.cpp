#include "fusion_ekf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = Eigen::MatrixXd(2, 2);
    R_radar_ = Eigen::MatrixXd(3, 3);
    H_laser_ = Eigen::MatrixXd(2, 4);
    Hj_ = Eigen::MatrixXd(3, 4);

    /**
    TODO:
      * Finish initializing the FusionEKF.
    */
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_)
    {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        std::cout << "EKF: " << std::endl;
        ekf_.x_ = Eigen::VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            /**
            Initialize state.
            */
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

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
    std::cout << "x_ = " << ekf_.x_ << std::endl;
    std::cout << "P_ = " << ekf_.P_ << std::endl;
}
