#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_

#include <vector>
#include <string>
#include <fstream>
#include <cstdint>
#include "kalman_filter.h"
#include "measurement_package.h"
#include "motion_model.h"
#include "measurement_model_lidar.h"
#include "measurement_model_radar.h"
#include "tools.h"

class FusionEKF
{
public:
    FusionEKF();

    /// \brief Runs one iteration of the Extended Kalman Filter given a
    ///        measurement
    /// \param measurement_pack incoming measurement
    void processMeasurement(const MeasurementPackage& measurement_pack);

    const Eigen::VectorXd& getState() const { return ekf_.getState(); }
    const Eigen::MatrixXd& getCovariance() const { return ekf_.getCovariance(); }

private:
    // Flag indicating whether the tracking toolbox was initialized or not
    bool is_initialized_;

    // Previous timestamp
    uint64_t previous_timestamp_;

    // Kalman Filter
    KalmanFilter ekf_;

    // Motion model (constant acceleration)
    MotionModel motion_model_;

    // Measurement models for lidar and radar
    MeasurementModelLidar meas_model_lidar_;
    MeasurementModelRadar meas_model_radar_;
};

#endif /* FUSION_EKF_H_ */
