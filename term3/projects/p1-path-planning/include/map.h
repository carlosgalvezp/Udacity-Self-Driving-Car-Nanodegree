#ifndef MAPDATA_H
#define MAPDATA_H

#include <vector>

#include "spline.h"

/// Struct containing the 181 waypoints that represent the map
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
    Map(const MapData& raw_data);

    /// \brief Transforms from (s,d) Frenet coordinates to (x,y) map coordinates
    /// \param s
    /// \param d
    /// \return a vector representing the
    std::pair<double, double> frenetToXy(const double s, const double d);

private:
    const MapData& raw_data_;

    tk::spline spline_x_;
    tk::spline spline_y_;
    tk::spline spline_dx_;
    tk::spline spline_dy_;
};

#endif // MAPDATA_H
