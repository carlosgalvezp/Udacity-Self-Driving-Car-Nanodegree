#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "sensor_fusion_data.h"
#include "ego_vehicle_data.h"
#include "map.h"

#include "behavior_planner.h"
#include "trajectory_generator.h"

/// High-level class responsible for generating a feasible
/// trajectory to send directly to the simulator, given
/// sensor data and a map.
class PathPlanner
{
public:
    PathPlanner();

    /// \brief Generates a trajectory in map (x,y) coordinates
    ///        for the vehicle to move forward
    /// \param ego_vehicle_data ego-vehicle data
    /// \param sensor_fusion_data sensor fusion data
    /// \param map map
    /// \param previous_x previous X points from the simulator
    /// \param previous_y previous Y points from the simulator
    /// \param out_x output X map coordinates to the simulator
    /// \param out_y output Y map coordinates to the simulator
    void generateTrajectory(const EgoVehicleData& ego_vehicle_data,
                            const SensorFusionData& sensor_fusion_data,
                            const Map& map,
                            const std::vector<double>& previous_x,
                            const std::vector<double>& previous_y,
                            std::vector<double>& out_x,
                            std::vector<double>& out_y);

private:
    BehaviorPlanner behavior_planner_;
    TrajectoryGenerator trajectory_generator_;
};

#endif // PATH_PLANNER_H
