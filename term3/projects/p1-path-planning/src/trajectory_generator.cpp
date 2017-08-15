#include "trajectory_generator.h"
#include <iostream>

#include "utils.h"

TrajectoryGenerator::TrajectoryGenerator()
{
}

void TrajectoryGenerator::generateTrajectory(const CarBehavior next_action,
                                             const EgoVehicleData& ego_vehicle_data,
                                             const MapData& map_data,
                                             const std::vector<double>& previous_x,
                                             const std::vector<double>& previous_y,
                                             std::vector<double>& out_x,
                                             std::vector<double>& out_y)
{
    switch (next_action)
    {
        case CarBehavior::GO_STRAIGHT:
            generateTrajectoryFollowLane(ego_vehicle_data, map_data, previous_x, previous_y, out_x, out_y);
            break;
        default:
            break;
    }
}

void TrajectoryGenerator::generateTrajectoryFollowLane(const EgoVehicleData& ego_vehicle_data,
                                                       const MapData& map_data,
                                                       const std::vector<double>& previous_x,
                                                       const std::vector<double>& previous_y,
                                                       std::vector<double>& out_x,
                                                       std::vector<double>& out_y)
{
    (void) ego_vehicle_data;
    (void) map_data;
    (void) previous_x;
    (void) previous_y;
    (void) out_x;
    (void) out_y;
}
