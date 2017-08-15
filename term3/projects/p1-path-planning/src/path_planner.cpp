#include "path_planner.h"

PathPlanner::PathPlanner():
    behavior_planner_()
{
}

void PathPlanner::generateTrajectory(const EgoVehicleData& ego_vehicle_data,
                                     const SensorFusionData& sensor_fusion_data,
                                     const Map &map,
                                     const std::vector<double>& previous_x,
                                     const std::vector<double>& previous_y,
                                     std::vector<double>& out_x,
                                     std::vector<double>& out_y)
{
    (void)sensor_fusion_data;

    // Decide next action
    CarBehavior next_action = behavior_planner_.getNextAction();

    // Generate trajectory
    trajectory_generator_.generateTrajectory(next_action,
                                             ego_vehicle_data,
                                             map,
                                             previous_x,
                                             previous_y,
                                             out_x, out_y);
}
