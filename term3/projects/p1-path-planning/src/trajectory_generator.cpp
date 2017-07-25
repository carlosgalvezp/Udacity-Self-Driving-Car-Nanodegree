#include "trajectory_generator.h"

#include "utils.h"

TrajectoryGenerator::TrajectoryGenerator()
{
}

void TrajectoryGenerator::generateTrajectory(const CarBehavior next_action,
                                             const EgoVehicleData& ego_vehicle_data,
                                             const MapData& map_data,
                                             std::vector<double>& out_x,
                                             std::vector<double>& out_y)
{
    switch (next_action)
    {
        case CarBehavior::GO_STRAIGHT:
            generateTrajectoryFollowLane(ego_vehicle_data, map_data, out_x, out_y);
            break;
        default:
            break;
    }
}

void TrajectoryGenerator::generateTrajectoryFollowLane(const EgoVehicleData& ego_vehicle_data,
                                                       const MapData& map_data,
                                                       std::vector<double>& out_x,
                                                       std::vector<double>& out_y)
{
    out_x.resize(kNrTrajectoryPoints);
    out_y.resize(kNrTrajectoryPoints);

    double dist_inc = 0.5;
    for(std::size_t i = 0; i < kNrTrajectoryPoints; ++i)
    {
        const double s_next = ego_vehicle_data.s + dist_inc * i;
        const double d_next = ego_vehicle_data.d;

        std::vector<double> xy_next = getXY(s_next, d_next, map_data.s, map_data.x, map_data.y);

        out_x[i] = xy_next[0U];
        out_y[i] = xy_next[1U];
    }
}
