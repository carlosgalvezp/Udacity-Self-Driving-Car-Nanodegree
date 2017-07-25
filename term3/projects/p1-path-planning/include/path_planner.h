#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "sensor_fusion_data.h"
#include "ego_vehicle_data.h"
#include "map_data.h"

class PathPlanner
{
public:
    PathPlanner();
    void generateTrajectory(const EgoVehicleData& ego_vehicle_data,
                            const SensorFusionData& sensor_fusion_data,
                            const MapData& map_data,
                            std::vector<double>& out_x,
                            std::vector<double>& out_y);

private:
    // The number of waypoints for the output trajectory
    static const std::size_t kNrTrajectoryPoints = 50U;

    // The delta time between waypoints, in seconds
    static constexpr double delta_t = 0.02;
};

#endif // PATH_PLANNER_H
