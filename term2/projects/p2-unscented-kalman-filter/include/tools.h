#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

/// \brief Set of common tools required for the project
class Tools
{
public:
    /// \brief Computes the Root Mean Squared Error (RMSE) between estimate and
    ///        ground truth
    /// \param estimations estimates
    /// \param ground_truth ground truth
    /// \return the computed RMSE
    static Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd>&
                                         estimations,
                                         const std::vector<Eigen::VectorXd>& ground_truth);

    /// \brief Returns true if x is not zero, up to some predefined threshold
    /// \param x input
    /// \return true if x is greater than a small threshold
    static bool isNotZero(const double x);

    /// \brief Normalizes an angle to the range [-pi, pi)
    /// \param x angle to normalize
    /// \return the normalized angle
    static double normalizeAngle(const double x);

    /// \brief Computes the square root of a matrix
    /// \param x the input matrix
    /// \return the square root of x, A, such that x = A' * A
    static Eigen::MatrixXd sqrt(const Eigen::MatrixXd& x);
};

#endif /* TOOLS_H_ */
