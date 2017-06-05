#ifndef TOOLS_H
#define TOOLS_H

#include <cmath>
#include <Eigen/Dense>
#include <cppad/cppad.hpp>

class Tools
{
public:
    /// \brief Evaluate a polynomial in the form of:
    ///        coeffs[0] + coeffs[1] * x + ... + coeffs[N] * x^N
    /// \param coeffs coefficients
    /// \param x evaluation point
    /// \return Evaluation of the polynomial y = p(x)
    static double polyeval(const Eigen::VectorXd& coeffs, double x);

    /// \brief Fit a polynomial.
    ///        Adapted from
    ///        https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    /// \param xvals x coordinates of the points defining the polynomial
    /// \param yvals y coordinates of the points defining the polynomial
    /// \param order order of the polynomial to fit
    /// \return coefficients of the polynomial
    ///
    static Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals,
                                   const Eigen::VectorXd& yvals,
                                   int order);

    // For converting back and forth between radians and degrees.
    static inline constexpr double pi() { return M_PI; }

    static inline constexpr double deg2rad(double x) { return x * pi() / 180.0; }
    static inline constexpr double rad2deg(double x) { return x * 180.0 / pi(); }

    static inline constexpr double mphtoms(double x) { return x * 0.44704; }
};

#endif // TOOLS_H
