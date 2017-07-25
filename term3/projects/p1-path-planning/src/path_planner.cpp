#include "path_planner.h"

#include <iostream>
#include <cmath>

#include "utils.h"

PathPlanner::PathPlanner()
{

}

void PathPlanner::generateTrajectory(const EgoVehicleData& ego_vehicle_data,
                                     const SensorFusionData& sensor_fusion_data,
                                     const MapData &map_data,
                                     std::vector<double> &out_x,
                                     std::vector<double> &out_y)
{
//    // Decide next action
//    action = behavior_planner.nextAction(ego_vehicle_data, sensor_fusion_data);

//    // Generate trajectory
//    trajectory_generator.generateTrajectory(action, sensor_fusion_data,
//                                            out_x, out_y);
    out_x.resize(kNrTrajectoryPoints);
    out_y.resize(kNrTrajectoryPoints);

    double dist_inc = 0.3;
    for(std::size_t i = 0; i < kNrTrajectoryPoints; ++i)
    {
        const double s_next = ego_vehicle_data.s + dist_inc * i;
        const double d_next = ego_vehicle_data.d;

        std::vector<double> xy_next = getXY(s_next, d_next, map_data.s, map_data.x, map_data.y);

        out_x[i] = xy_next[0U];
        out_y[i] = xy_next[1U];
    }
}
