#ifndef MAPDATA_H
#define MAPDATA_H

#include <vector>

/// Struct containing the 181 waypoints that represent the map
struct MapData
{
    std::vector<double> x;   ///< X coordinates
    std::vector<double> y;   ///< Y coordinates
    std::vector<double> s;   ///< s Frenet coordinates
    std::vector<double> dx;  ///< normalized d vector, pointing to right of road (X component)
    std::vector<double> dy;  ///< normalized d vector, pointing to right of road (Y component)
};

#endif // MAPDATA_H
