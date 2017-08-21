#include "utils.h"

#include <limits>

#include <Eigen/Dense>

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string& s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

void generateJerkMinTrajectory(const double x0, const double x0_d, const double x0_dd,
                               const double xf, const double xf_d, const double xf_dd,
                               const double t,
                               std::vector<double>& trajectory_coeffs)
{
    // Define output
    const std::size_t kNrCoeffs = 6U;  // 5th-order polynomial
    trajectory_coeffs.resize(kNrCoeffs);

    // Compute the last coefficients
    const double t2 = t * t;
    const double t3 = t * t2;
    const double t4 = t * t3;
    const double t5 = t * t4;

    Eigen::Matrix3d A;
    A <<     t3,       t4,        t5,
         3.0*t2,   4.0*t3,    5.0*t4,
         6.0*t,   12.0*t2,   20.0*t3;

    Eigen::Vector3d b;
    b << xf    - (x0 + x0_d*t + 0.5*x0_dd*t2),
         xf_d  - (     x0_d   +     x0_dd*t),
         xf_dd - (                  x0_dd);

    Eigen::Vector3d a3a4a5 = A.inverse() * b;

    // Set output
    trajectory_coeffs[0U] = x0;
    trajectory_coeffs[1U] = x0_d;
    trajectory_coeffs[2U] = 0.5 * x0_dd;
    trajectory_coeffs[3U] = a3a4a5[0U];
    trajectory_coeffs[4U] = a3a4a5[1U];
    trajectory_coeffs[5U] = a3a4a5[2U];
}

void generateJerkMinTrajectory(const double x0, const double x0_d, const double x0_dd,
                               const double xf_d, const double xf_dd,
                               const double t,
                               std::vector<double>& trajectory_coeffs)
{
    // Define output
    const std::size_t kNrCoeffs = 5U;  // 4th-order polynomial
    trajectory_coeffs.resize(kNrCoeffs);

    // Compute the last coefficients
    const double t2 = t * t;
    const double t3 = t * t2;

    Eigen::Matrix2d A;
    A << 3.0*t2,   4.0*t3,
         6.0*t,   12.0*t2;

    Eigen::Vector2d b;
    b << xf_d  - (     x0_d   +     x0_dd*t),
         xf_dd - (                  x0_dd);

    Eigen::Vector2d a3a4 = A.inverse() * b;

    // Set output
    trajectory_coeffs[0U] = x0;
    trajectory_coeffs[1U] = x0_d;
    trajectory_coeffs[2U] = 0.5 * x0_dd;
    trajectory_coeffs[3U] = a3a4[0U];
    trajectory_coeffs[4U] = a3a4[1U];
}

void generateJerkMinTrajectory(const double x0, const double x0_d,
                               const double xf_d, const double t,
                               std::vector<double>& trajectory_coeffs)
{
    // Define output
    const std::size_t kNrCoeffs = 3U;  // 2nd-order polynomial
    trajectory_coeffs.resize(kNrCoeffs);

    // Set output
    trajectory_coeffs[0U] = x0;
    trajectory_coeffs[1U] = x0_d;
    trajectory_coeffs[2U] = (xf_d - x0_d) / (2.0 * t);
}

double evaluatePolynomial(const std::vector<double>& coeffs, const double x)
{
    double result = 0.0;
    for (std::size_t i = 0U; i < coeffs.size(); ++i)
    {
        result += coeffs[i] * std::pow(x, static_cast<double>(i));
    }

    return result;
}
