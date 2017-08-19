#include "trajectory_generator.h"
#include <iostream>

#include "utils.h"

TrajectoryGenerator::TrajectoryGenerator():
    previous_s_(),
    previous_d_()
{
}

void TrajectoryGenerator::generateTrajectory(const CarBehavior next_action,
                                             const EgoVehicleData& ego_vehicle_data,
                                             const Map &map,
                                             const std::vector<double>& previous_x,
                                             const std::vector<double>& previous_y,
                                             std::vector<double>& out_x,
                                             std::vector<double>& out_y)
{
    // Initialize output
    out_x.clear();
    out_y.clear();

    // Remove the points that the simulator consumed in the last iteration
    // from previous_s_ and previous_d_
    const std::size_t n_points_consumed = kNrTrajectoryPoints - previous_x.size();
    for (std::size_t i = 0U; i < n_points_consumed; ++i)
    {
        if (!previous_s_.empty() && !previous_d_.empty())
        {
            previous_s_.pop_front();
            previous_d_.pop_front();
        }
        else
        {
            break;
        }
    }

    // Number of points to keep from the previous trajectory
    const std::size_t n_points_keep = std::min(previous_x.size(), kNrPreviousPathPoints);
    previous_s_.resize(n_points_keep);
    previous_d_.resize(n_points_keep);

    // Copy part of the previous trajectory to the output
    for (std::size_t i = 0U; i < n_points_keep; ++i)
    {
        out_x.push_back(previous_x[i]);
        out_y.push_back(previous_y[i]);
    }

    // Get ego-vehicle data in Frenet coordinates, where we start generating
    // the new trajectory
    EgoVehicleFrenet ego_vehicle_frenet = EgoVehicleFrenet();
    if (previous_x.size() < 2U)
    {
        ego_vehicle_frenet.s = ego_vehicle_data.s;
        ego_vehicle_frenet.d = ego_vehicle_data.d;
    }
    else
    {
        ego_vehicle_frenet = getEgoVehicleFrenetFromPreviousTrajectory();
    }

    // Generate new trajectory
    const std::size_t n_new_points = kNrTrajectoryPoints - n_points_keep;
    EgoVehicleFrenet next_state = EgoVehicleFrenet();

    next_state.s_dot = kRoadSpeedLimit;

    switch (next_action)
    {
        case CarBehavior::GO_STRAIGHT:
            next_state.d = (Map::getLaneNumber(ego_vehicle_frenet.d) + 0.5) * kLaneWidth;
            break;
        case CarBehavior::CHANGE_LANE_LEFT:
            next_state.d = (Map::getLaneNumber(ego_vehicle_frenet.d) - 0.5) * kLaneWidth;
            break;

        case CarBehavior::CHANGE_LANE_RIGHT:
            next_state.d = (Map::getLaneNumber(ego_vehicle_frenet.d) + 1.5) * kLaneWidth;
            break;
        default:
            break;
    }

    generateTrajectoryFollowLane(ego_vehicle_frenet, next_state, map, n_new_points, out_x, out_y);
}


EgoVehicleFrenet TrajectoryGenerator::getEgoVehicleFrenetFromPreviousTrajectory()
{
    EgoVehicleFrenet output;

    const std::size_t last_index = previous_s_.size() - 1U;

    output.s = previous_s_[last_index];
    output.s_dot = estimateVelocity(previous_s_, last_index, kSimulationTimeStep);
    output.s_ddot = estimateAcceleration(previous_s_, last_index, kSimulationTimeStep);

    output.d = previous_d_[last_index];
    output.d_dot = estimateVelocity(previous_d_, last_index, kSimulationTimeStep);
    output.d_ddot = estimateAcceleration(previous_d_, last_index, kSimulationTimeStep);

    return output;
}

double TrajectoryGenerator::estimateVelocity(const std::deque<double>& trajectory,
                                             std::size_t index, double dt)
{
    double output = 0.0;

    if ((index < 1U) || (index >= trajectory.size()))
    {
        std::cerr << "Invalid input arguments to estimate velocity."
                  << "Trajectory size: " << trajectory.size()
                  << ", index: " << index;
    }
    else
    {
        output = Map::s_diff(trajectory[index], trajectory[index - 1U]) / dt;
    }
    return output;
}

double TrajectoryGenerator::estimateAcceleration(const std::deque<double>& trajectory,
                                                 std::size_t index, double dt)
{
    double output = 0.0;

    if ((index < 2U) || (index >= trajectory.size()))
    {
        std::cerr << "Invalid input arguments to estimate acceleration."
                  << "Trajectory size: " << trajectory.size()
                  << ", index: " << index;
    }
    else
    {
        const double v1 = estimateVelocity(trajectory, index,      dt);
        const double v2 = estimateVelocity(trajectory, index - 1U, dt);

        output = (v1 - v2) / dt;
    }
    return output;
}

void TrajectoryGenerator::generateTrajectoryFollowLane(const EgoVehicleFrenet& ego_vehicle_data,
                                                       const EgoVehicleFrenet& target_state,
                                                       const Map& map,
                                                       const std::size_t n_new_points,
                                                       std::vector<double>& out_x,
                                                       std::vector<double>& out_y)
{
    // Create JMT trajectory
    double s0, d0;
    if (previous_s_.empty())
    {
        s0 = ego_vehicle_data.s;
        d0 = ego_vehicle_data.d;
    }
    else
    {
        s0 = previous_s_.back();
        d0 = previous_d_.back();
    }

    // s-trajectory
    std::vector<double> coeffs_s;
    if (target_state.s == 0.0)
    {
        // s-Trajectory based on velocity
        const double v_max = ego_vehicle_data.s_dot + kMaxAcceleration * kTrajectoryDuration;
        const double v_target = std::min(v_max, target_state.s_dot);

//        generateJerkMinTrajectory(s0, ego_vehicle_data.s_dot, 0.0,
//                                  v_target, 0.0, kTrajectoryDuration, coeffs_s);
        generateJerkMinTrajectory(s0, ego_vehicle_data.s_dot,
                                  v_target, kTrajectoryDuration, coeffs_s);
    }
    else
    {
        // s-Trajectory based on position
        generateJerkMinTrajectory(s0, ego_vehicle_data.s_dot, 0.0,
                                  target_state.s, target_state.s_dot, 0.0, kTrajectoryDuration, coeffs_s);
    }

    // d-Trajectory - always based on position
    std::vector<double> coeffs_d;
    generateJerkMinTrajectory(d0, ego_vehicle_data.d_dot, 0.0,
                              target_state.d, 0.0, 0.0, kTrajectoryDuration, coeffs_d);

    // Create spatial trajectory
    for (std::size_t i = 0U; i < n_new_points; ++i)
    {
        const double t = static_cast<double>(i+1U) * kSimulationTimeStep;
        const double s = std::fmod(evaluatePolynomial(coeffs_s, t), kMaxS);
        const double d =           evaluatePolynomial(coeffs_d, t);

        // Store it for future reference
        previous_s_.push_back(s);
        previous_d_.push_back(d);

        // Output
        std::pair<double, double> xy = map.frenetToXy(s, d);

        out_x.push_back(xy.first);
        out_y.push_back(xy.second);
    }
}
