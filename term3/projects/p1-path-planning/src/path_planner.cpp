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
    double dist_inc = 0.5;
    for(std::size_t i = 0; i < kNrTrajectoryPoints; ++i)
    {
        out_x[i] = ego_vehicle_data.x + (dist_inc*i)*std::cos(deg2rad(ego_vehicle_data.yaw));
        out_y[i] = ego_vehicle_data.y + (dist_inc*i)*std::sin(deg2rad(ego_vehicle_data.yaw));
    }
}
