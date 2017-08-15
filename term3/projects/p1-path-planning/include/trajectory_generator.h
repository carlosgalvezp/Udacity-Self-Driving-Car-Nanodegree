#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>

#include "ego_vehicle_data.h"
#include "map.h"

#include "behavior_planner.h"

// The delta time between waypoints, in seconds
const double kSimulationTimeStep = 0.02;

// The temporal length of the trajectory, in seconds
const double kTrajectoryDuration = 2.0;

// The number of waypoints for the output trajectory
const std::size_t kNrTrajectoryPoints = kTrajectoryDuration / kSimulationTimeStep;

class TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    void generateTrajectory(const CarBehavior next_action,
                            const EgoVehicleData& ego_vehicle_data,
                            const Map& map,
                            const std::vector<double>& previous_x,
                            const std::vector<double>& previous_y,
                            std::vector<double>& out_x,
                            std::vector<double>& out_y);
private:
    void generateTrajectoryFollowLane(const EgoVehicleData& ego_vehicle_data,
                                      const Map& map,
                                      const std::vector<double>& previous_x,
                                      const std::vector<double>& previous_y,
                                      std::vector<double>& out_x,
                                      std::vector<double>& out_y);
};

#endif // TRAJECTORY_GENERATOR_H
