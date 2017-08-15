#include "trajectory_generator.h"
#include <iostream>

#include "utils.h"

TrajectoryGenerator::TrajectoryGenerator()
{
}

void TrajectoryGenerator::generateTrajectory(const CarBehavior next_action,
                                             const EgoVehicleData& ego_vehicle_data,
                                             const Map &map,
                                             const std::vector<double>& previous_x,
                                             const std::vector<double>& previous_y,
                                             std::vector<double>& out_x,
                                             std::vector<double>& out_y)
{
    switch (next_action)
    {
        case CarBehavior::GO_STRAIGHT:
            generateTrajectoryFollowLane(ego_vehicle_data, map, previous_x, previous_y, out_x, out_y);
            break;
        default:
            break;
    }
}

void TrajectoryGenerator::generateTrajectoryFollowLane(const EgoVehicleData& ego_vehicle_data,
                                                       const Map &map,
                                                       const std::vector<double>& previous_x,
                                                       const std::vector<double>& previous_y,
                                                       std::vector<double>& out_x,
                                                       std::vector<double>& out_y)
{
    (void) previous_x;
    (void) previous_y;

    out_x.resize(kNrTrajectoryPoints);
    out_y.resize(kNrTrajectoryPoints);

    for (std::size_t i = 0U; i < kNrTrajectoryPoints; ++i)
    {
        const double s = ego_vehicle_data.s + static_cast<double>(i + 1U) * 0.3;
        const double d = 6.0;

        std::pair<double, double> xy = map.frenetToXy(s, d);

        out_x.push_back(xy.first);
        out_y.push_back(xy.second);
    }
}
