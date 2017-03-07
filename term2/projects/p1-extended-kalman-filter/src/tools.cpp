#include "tools.h"

Eigen::VectorXd Tools::calculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                     const std::vector<Eigen::VectorXd>& ground_truth)
{
    Eigen::VectorXd output = Eigen::VectorXd::Zero(estimations[0].rows(),
                                                   estimations[0].cols());

    for (std::size_t i = 0U; i < estimations.size(); ++i)
    {
        Eigen::VectorXd error = ground_truth[i] - estimations[i];
        output += error.cwiseProduct(error);
    }

    output /= estimations.size();

    return output.cwiseSqrt();
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& /*x_state*/)
{
    Eigen::MatrixXd x;
    // TODO
    return x;
}
