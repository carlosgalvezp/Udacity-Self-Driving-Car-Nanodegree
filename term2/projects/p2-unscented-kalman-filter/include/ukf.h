#ifndef UKF_H
#define UKF_H
#include "Eigen/Dense"
#include "measurement_package.h"
#include <vector>

class UKF
{
public:
    explicit UKF(const std::size_t n_states);

    const Eigen::VectorXd& getState() const { return x_; }
    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);
private:
    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);


    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    /// State vector: [pos_x pos_y velocity yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    ///* state covariance matrix
    Eigen::MatrixXd P_;

    ///* predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;

    ///* Weights of sigma points
    Eigen::VectorXd weights_;

    ///* time when the state is true, in us
    long time_us_;



    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;

    ///* the current NIS for radar
    double NIS_radar_;

    ///* the current NIS for laser
    double NIS_laser_;
};

#endif /* UKF_H */
