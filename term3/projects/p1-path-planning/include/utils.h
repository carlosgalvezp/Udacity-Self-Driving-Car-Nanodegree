#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <cmath>
#include <utility>

#include "map.h"

constexpr double pi() { return M_PI; }
constexpr double deg2rad(const double x) { return x * pi() / 180.0; }
constexpr double rad2deg(double x) { return x * 180.0 / pi(); }
constexpr double mph2ms(double x) { return x * 0.44704; }

std::string hasData(const std::string& s);

/// \brief Computes the coefficients of a jerk-minimizing 5-order polynomial
///        trajectory, given the initial and final conditions, as well as
///        the time taken to go from initial to final states.
/// \param x0       initial position
/// \param x0_d     initial velocity
/// \param x0_dd    initial acceleration
/// \param xf       final position
/// \param xf_d     final velocity
/// \param xf_dd    final acceleration
/// \param t        time to go from initial to final state
/// \param trajectory_coeffs coefficients of the trajectory, in the form:
///
///        x(t) = c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 + c[4]*t^4 + c[5]*t^5
void generateJerkMinTrajectory(const double x0, const double x0_d, const double x0_dd,
                               const double xf, const double xf_d, const double xf_dd,
                               const double t,
                               std::vector<double>& trajectory_coeffs);

void generateJerkMinTrajectory(const double x0, const double x0_d, const double x0_dd,
                               const double xf_d, const double xf_dd,
                               const double t,
                               std::vector<double>& trajectory_coeffs);

/// \brief Evaluates the polynomial:
///
///        f(x) = coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + ...
///
/// \param coeffs coefficients of the polynomial
/// \param x      point at which the polynomial is evaluated
/// \return f(x), the polynomial evaluated at x
double evaluatePolynomial(const std::vector<double>& coeffs, const double x);

/// \brief Differentiates once a polynomial
/// \param coeffs coefficients of the polynomial, f(x)
/// \return coefficients of the differentiated polynomial, f'(x)
std::vector<double> differentiatePolynomial(const std::vector<double>& coeffs);

#endif // UTILS_H
