#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools
{
public:
    /// \brief computes the Root Mean Squared Error (RMSE) between estimate and
    ///        ground truth
    /// \param estimations estimates
    /// \param ground_truth ground truth
    /// \return the computed RMSE
    static Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                         const std::vector<Eigen::VectorXd>& ground_truth);

    /// \brief Returns true if x is not zero, up to some predefined threshold
    /// \param x input
    /// \return true if x is greater than a small threshold
    static bool isNotZero(const double x);
};

#endif /* TOOLS_H_ */
