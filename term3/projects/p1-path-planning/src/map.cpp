#include "map.h"

Map::Map(const MapData &raw_data) :
    raw_data_(raw_data)
{
    spline_x_.set_points(raw_data_.s, raw_data_.x);
    spline_y_.set_points(raw_data_.s, raw_data_.y);
    spline_dx_.set_points(raw_data_.s, raw_data_.dx);
    spline_dy_.set_points(raw_data_.s, raw_data_.dy);
}

std::pair<double, double> Map::frenetToXy(const double s, const double d) const
{
    // Compute (x,y) position in the center of the road
    const double x_center = spline_x_(s);
    const double y_center = spline_y_(s);

    // Get d vector
    const double dx = spline_dx_(s);
    const double dy = spline_dy_(s);

    // Compute final position adding offset in the d direction
    const double x_out = x_center + d * dx;
    const double y_out = y_center + d * dy;

    return {x_out, y_out};
}
