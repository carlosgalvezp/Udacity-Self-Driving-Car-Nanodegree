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

    /// \brief Initializes the filter with the first measurement
    /// \param measurement_pack
    void initialize(const MeasurementPackage& measurement_pack);

    /// \brief Runs one iteration of the Extended Kalman Filter given a
    ///        measurement
    /// \param measurement_pack incoming measurement
    void processMeasurement(const MeasurementPackage& measurement_pack);

    /// \brief Returns the current state
    /// \return the current state
    const Eigen::VectorXd& getState() const { return ekf_.getState(); }

private:
    // Flag indicating whether the tracking toolbox was initialized or not
    bool is_initialized_;

    // Previous timestamp
    long previous_timestamp_;

    // Kalman Filter
    KalmanFilter ekf_;

    // Motion model (constant acceleration)
    MotionModel motion_model_;

    // Measurement models for lidar and radar
    MeasurementModelLidar meas_model_lidar_;
    MeasurementModelRadar meas_model_radar_;

    // Dimension of the state
    static const std::size_t n_states_ = 4U;
};

#endif /* FUSION_EKF_H_ */
