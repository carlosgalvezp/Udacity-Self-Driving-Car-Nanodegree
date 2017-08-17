#include "map.h"

Map::Map(const MapData &raw_data) :
    raw_data_(raw_data)
{
    // Push an extra point at s = max_s, to ensure continuity
    raw_data_.s.push_back(kMaxS);
    raw_data_.x.push_back(raw_data_.x[0U]);
    raw_data_.y.push_back(raw_data_.y[0U]);
    raw_data_.dx.push_back(raw_data_.dx[0U]);
    raw_data_.dy.push_back(raw_data_.dy[0U]);

    // Compute splines
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
