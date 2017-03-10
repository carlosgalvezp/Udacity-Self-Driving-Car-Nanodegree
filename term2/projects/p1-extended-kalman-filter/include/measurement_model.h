#ifndef MEASUREMENTMODEL_H
#define MEASUREMENTMODEL_H

#include <Eigen/Dense>

class MeasurementModel
{
public:
    MeasurementModel();
    virtual ~MeasurementModel();

    Eigen::MatrixXd R_;
    Eigen::MatrixXd H_;
private:

};

#endif // MEASUREMENTMODEL_H
