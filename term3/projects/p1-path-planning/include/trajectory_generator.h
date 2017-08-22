#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <deque>

#include "ego_vehicle_data.h"
#include "egovehiclefrenet.h"

#include "map.h"
#include "sensor_fusion_data.h"
#include "behavior_planner.h"
#include "utils.h"

/// The delta time between waypoints, in seconds
const double kSimulationTimeStep = 0.02;

/// The temporal length of the trajectory, in seconds
const double kTrajectoryDuration = 2.0;

/// The number of waypoints for the output trajectory
const std::size_t kNrTrajectoryPoints = static_cast<std::size_t>(kTrajectoryDuration / kSimulationTimeStep);

/// The reaction time of the vehicle, in seconds.
// It represents for how much time the vehicle follows the previous planned
// trajectory, before using a new trajectory based on the Sensor Fusion
// information.
// A large value ensures that the trajectory is smooth.
// However, making it too large will make it slow to react (for example,
// when another vehicle brakes)
const double kReactionTime = 0.1;

/// The number of points to keep from the previous path
const std::size_t kNrPreviousPathPoints = static_cast<std::size_t>(kReactionTime / kSimulationTimeStep);

/// Maximum acceleration
const double kMaxAcceleration = 9.5;  // [m/s^2]

/// Target speed for each lane [left, center, right]
/// Required since Frenet coordinates are not accurate and we end up
/// getting actual velocities over the speed limit in the simulator.
/// This has more effect on the right-most lane, thus the smaller
/// target velocity.
const std::vector<double> kTargetLaneSpeed = {mph2ms(48.5), mph2ms(47.0), mph2ms(46.0)};  // [m/s]

/// We stop tracking the vehicle in front of us if the gap is larger than this
const double kTargetTrackingMaxGap = 30.0;      // [m]

/// We slow down if the distance to the next vehicle is smaller than this
const double kTargetTrackingMinGap = 10.0;      // [m]

/// Low-level class responsible for generating a trajectory, given
/// the next desired action, and data from the ego-vehicle and its environment
class TrajectoryGenerator
{
public:
    TrajectoryGenerator();

    /// \brief Generates a trajectory given the best action to follow
    /// \param next_action  action to take
    /// \param ego_vehicle_data ego-vehicle data
    /// \param sensor_fusion sensor fusion data
    /// \param map map data
    /// \param previous_x previous X points, from the simulator
    /// \param previous_y previous Y points, from the simulator
    /// \param out_x output X map coordinates, to send to the simulator
    /// \param out_y output Y map coordinates, to send to the simulator
    void generateTrajectory(const CarBehavior next_action,
                            const EgoVehicleData& ego_vehicle_data,
                            const SensorFusionData& sensor_fusion,
                            const Map& map,
                            const std::vector<double>& previous_x,
                            const std::vector<double>& previous_y,
                            std::vector<double>& out_x,
                            std::vector<double>& out_y);
private:
    EgoVehicleFrenet getEgoVehicleFrenetFromPreviousTrajectory();
    void generateTrajectoryFollowLane(const EgoVehicleFrenet& ego_vehicle_data,
                                      const EgoVehicleFrenet& target_state,
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

    double target_d_for_lane_change_;
};

#endif // TRAJECTORY_GENERATOR_H
