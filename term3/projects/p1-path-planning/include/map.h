#ifndef MAPDATA_H
#define MAPDATA_H

#include <string>
#include <vector>
#include <cmath>

#include "spline.h"
#include "utils.h"

/// The length of the circuit
const double kMaxS = 6945.554;  // [m]

/// The lane width
const double kLaneWidth = 4.0;  // [m]

/// The speed limit of the road
const double kRoadSpeedLimit = mph2ms(50.0);  // [m/s]

/// Struct containing the waypoints that represent the map
struct MapData
{
    /// Constructor
    /// \param Path to the CSV file to read data from
    MapData(const std::string& map_csv_path);

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

    /// \brief Computes the smallest difference between two 's' Frenet coordinates:
    ///        output = s_a - s_b
    ///        Taking into account that the circuit is circular and therefore
    ///        there is a wrapping between the final and initial s coordinate.
    /// \param s_a first s coordinate
    /// \param s_b second s coordinate
    /// \return s_a - s_b, wrapped around the circuit length
    static double s_min_diff(double s_a, double s_b)
    {
        double output = s_a - s_b;

        if (output < -0.5 * kMaxS)
        {
            output += kMaxS;
        }
        else if (output > 0.5 * kMaxS)
        {
            output -= kMaxS;
        }

        return output;
    }

    /// \brief Returns the lane number given the 'd' Frenet coordinate
    /// \param d d Frenet coordinate
    /// \return an integer representing the ID of the lane
    ///         it will be in the range [-2, 2]
    static int getLaneNumber(double d)
    {
        return std::floor(d / kLaneWidth);
    }

private:
    MapData raw_data_;

    tk::spline spline_x_;
    tk::spline spline_y_;
    tk::spline spline_dx_;
    tk::spline spline_dy_;
};

#endif // MAPDATA_H
