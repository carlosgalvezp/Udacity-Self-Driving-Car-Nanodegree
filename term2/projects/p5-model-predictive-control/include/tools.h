#ifndef TOOLS_H
#define TOOLS_H

#include <cmath>
#include <Eigen/Dense>

class Tools
{
    // Evaluate a polynomial.
    static double polyeval(const Eigen::VectorXd& coeffs, double x);

    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    static Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals,
                                   const Eigen::VectorXd& yvals,
                                   int order);

    // For converting back and forth between radians and degrees.
    static inline constexpr double pi() { return M_PI; }
    static inline double deg2rad(double x) { return x * pi() / 180; }
    static inline double rad2deg(double x) { return x * 180 / pi(); }
};

#endif // TOOLS_H
