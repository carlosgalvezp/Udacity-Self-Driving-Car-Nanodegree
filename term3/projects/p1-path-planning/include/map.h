#ifndef MAPDATA_H
#define MAPDATA_H

#include <vector>

#include "spline.h"

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

private:
    const MapData& raw_data_;

    tk::spline spline_x_;
    tk::spline spline_y_;
    tk::spline spline_dx_;
    tk::spline spline_dy_;
};

#endif // MAPDATA_H
