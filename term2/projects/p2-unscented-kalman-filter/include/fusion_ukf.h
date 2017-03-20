#ifndef FUSION_UKF_H
#define FUSION_UKF_H

#include <Eigen/Dense>
#include "measurement_package.h"
#include "ukf.h"

class FusionUKF
{
public:
    FusionUKF();

    void processMeasurement(const MeasurementPackage& measurement);
    const Eigen::VectorXd& getState() const { return ukf_.getState(); }

private:
    UKF ukf_;
};

#endif // FUSION_UKF_H
