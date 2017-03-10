#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <Eigen/Dense>

class MeasurementModel
{
public:
    MeasurementModel();
    virtual ~MeasurementModel();

    const Eigen::MatrixXd& getMeasurementMatrix() const { return H_; }
    const Eigen::MatrixXd& getMeasurementNoise() const { return R_; }

    Eigen::MatrixXd R_;
    Eigen::MatrixXd H_;
private:

};

#endif // MEASUREMENTMODEL_H
