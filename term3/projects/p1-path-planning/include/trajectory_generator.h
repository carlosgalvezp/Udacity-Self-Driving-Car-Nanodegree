#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <deque>

#include "ego_vehicle_data.h"
#include "egovehiclefrenet.h"

#include "map.h"

#include "behavior_planner.h"

// The delta time between waypoints, in seconds
const double kSimulationTimeStep = 0.02;

// The temporal length of the trajectory, in seconds
const double kTrajectoryDuration = 2.0;

// The number of waypoints for the output trajectory
const std::size_t kNrTrajectoryPoints = static_cast<std::size_t>(kTrajectoryDuration / kSimulationTimeStep);

// The reaction time of the vehicle, in seconds.
// It represents for how much time the vehicle follows the previous planned
// trajectory, before using a new trajectory based on the Sensor Fusion
// information.
// A large value ensures that the trajectory is smooth.
// However, making it too large will make it slow to react (for example,
// when another vehicle breaks)
const double kReactionTime = 0.2;

// The number of points to keep from the previous path
const std::size_t kNrPreviousPathPoints = static_cast<std::size_t>(kReactionTime / kSimulationTimeStep);

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
    EgoVehicleFrenet getEgoVehicleFrenetFromPreviousTrajectory(const std::size_t last_index);
    void generateTrajectoryFollowLane(const EgoVehicleFrenet& ego_vehicle_data,
                                      const Map& map,
                                      const std::size_t n_new_points,
                                      std::vector<double>& out_x,
                                      std::vector<double>& out_y);

    double estimateVelocity(const std::deque<double> &trajectory,
                            std::size_t index, double dt);

    double estimateAcceleration(const std::deque<double> &trajectory,
                                std::size_t index, double dt);

    std::deque<double> previous_s_;
    std::deque<double> previous_d_;
};

#endif // TRAJECTORY_GENERATOR_H
