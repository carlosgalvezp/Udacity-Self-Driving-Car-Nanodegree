#ifndef MAPDATA_H
#define MAPDATA_H

#include <vector>
#include <cmath>

#include "spline.h"

constexpr double kMaxS = 6945.554;  // [m]

/// Struct containing the waypoints that represent the map
struct MapData
{
    std::vector<double> x;   ///< X coordinates
    std::vector<double> y;   ///< Y coordinates
    std::vector<double> s;   ///< s Frenet coordinates
    std::vector<double> dx;  ///< normalized d vector, pointing to right of road (X component)
    std::vector<double> dy;  ///< normalized d vector, pointing to right of road (Y component)
};

/// Class providing information and utility functions related to the map
class Map
{
public:
    /// \brief Constructor
    /// \param raw_data reference to the raw map data
    Map(const MapData& raw_data);

    /// \brief Transforms from (s,d) Frenet coordinates to (x,y) map coordinates
    /// \param s the s component of the Frenet coordinate
    /// \param d the d component of the Frenet coordinate
    /// \return the (x,y) map coordinates, as a pair
    std::pair<double, double> frenetToXy(const double s, const double d) const;

    /// \brief Computes the difference between two 's' Frenet coordinates:
    ///        output = s_a - s_b
    ///        Taking into account that the circuit is circular and therefore
    ///        there is a wrapping between the final and initial s coordinate.
    /// \param s_a first s coordinate
    /// \param s_b second s coordinate
    /// \return s_a - s_b, wrapped around the circuit length
    static constexpr double s_diff(double s_a, double s_b)
    {
        return std::fmod(s_a - s_b, kMaxS);
    }

private:
    const MapData& raw_data_;

    tk::spline spline_x_;
    tk::spline spline_y_;
    tk::spline spline_dx_;
    tk::spline spline_dy_;
};

#endif // MAPDATA_H
