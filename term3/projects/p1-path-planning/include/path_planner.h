#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "sensor_fusion_data.h"
#include "ego_vehicle_data.h"
#include "map_data.h"

#include "behavior_planner.h"
#include "trajectory_generator.h"

class PathPlanner
{
public:
    PathPlanner();
    void generateTrajectory(const EgoVehicleData& ego_vehicle_data,
                            const SensorFusionData& sensor_fusion_data,
                            const MapData& map_data,
                            const std::vector<double>& previous_x,
                            const std::vector<double>& previous_y,
                            std::vector<double>& out_x,
                            std::vector<double>& out_y);

private:
    BehaviorPlanner behavior_planner_;
    TrajectoryGenerator trajectory_generator_;
};

#endif // PATH_PLANNER_H
