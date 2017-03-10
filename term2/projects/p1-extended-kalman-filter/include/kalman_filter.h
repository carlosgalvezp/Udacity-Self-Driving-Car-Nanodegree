#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "motion_model.h"
#include "measurement_model.h"

class KalmanFilter
{
public:

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transistion matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Hj_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    Eigen::MatrixXd I_;

    KalmanFilter();
    ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
              Eigen::MatrixXd& F_in, Eigen::MatrixXd& H_in,
              Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void predict(const MotionModel& motion_model, const double delta_t);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void update(const Eigen::VectorXd& z);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void updateEKF(const Eigen::VectorXd& z);
};

#endif /* KALMAN_FILTER_H_ */
