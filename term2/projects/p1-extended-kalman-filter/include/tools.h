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

    /// \brief Computes the Jacobian matrix given the current state
    /// \param x_state current state
    /// \return Jacobian matrix associated with the given state
    static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
};

#endif /* TOOLS_H_ */
