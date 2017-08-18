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

    // Number of points that the simulator consummed in the last iteration
    const std::size_t n_points_consummed = kNrTrajectoryPoints - previous_x.size();
    for (std::size_t i = 0U; i < n_points_consummed; ++i)
    {
        if (!previous_s_.empty() && !previous_d_.empty())
        {
            previous_s_.pop_front();
            previous_d_.pop_front();
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
        ego_vehicle_frenet = getEgoVehicleFrenetFromPreviousTrajectory(n_points_keep - 1U);
    }

    // Generate new trajectory
    const std::size_t n_new_points = kNrTrajectoryPoints - n_points_keep;

    switch (next_action)
    {
        case CarBehavior::GO_STRAIGHT:
            generateTrajectoryFollowLane(ego_vehicle_frenet, map, n_new_points, out_x, out_y);
            break;
        default:
            break;
    }
}


EgoVehicleFrenet TrajectoryGenerator::getEgoVehicleFrenetFromPreviousTrajectory(const std::size_t last_index)
{
    EgoVehicleFrenet output;

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
        const double v1 = estimateVelocity(trajectory, index, dt);
        const double v2 = estimateVelocity(trajectory, index - 1U, dt);

        output = (v2 - v1) / dt;
    }
    return output;
}

void TrajectoryGenerator::generateTrajectoryFollowLane(const EgoVehicleFrenet& ego_vehicle_data,
                                                       const Map& map,
                                                       const std::size_t n_new_points,
                                                       std::vector<double>& out_x,
                                                       std::vector<double>& out_y)
{
//    // First, compute the desired velocity at every point of the trajectory
//    const double achievable_velocity = ego_vehicle_data.s_dot + kMaxAcceleration * kTrajectoryDuration;
//    const double target_velocity = std::min(achievable_velocity, kRoadSpeedLimit);

//    std::vector<double> s_dot_trajectory(n_new_points);
//    s_dot_trajectory[0U] = std::min(ego_vehicle_data.s_dot + kMaxAcceleration * kSimulationTimeStep, target_velocity);
//    for (std::size_t i = 1U; i < n_new_points; ++i)
//    {
//        s_dot_trajectory[i] = std::min(s_dot_trajectory[i - 1U] + kMaxAcceleration * kSimulationTimeStep, target_velocity);
//    }

    // Create JMT trajectory
    std::vector<double> coeffs;
    double s0;
    if (previous_s_.empty())
    {
        s0 = ego_vehicle_data.s;
    }
    else
    {
        s0 = previous_s_.back();
    }

    const double v_target = mph2ms(49.5);
    const double avg_v = 0.5 * (ego_vehicle_data.s_dot + v_target);

    generateJerkMinTrajectory(s0, ego_vehicle_data.s_dot, 0.0,
                              s0+avg_v * kTrajectoryDuration, v_target, 0.0, kTrajectoryDuration, coeffs);

    std::cout << ego_vehicle_data.s_dot << std::endl;
    // Create spatial trajectory
    for (std::size_t i = 0U; i < n_new_points; ++i)
    {
        const double s = evaluatePolynomial(coeffs, static_cast<double>(i+1U) * kSimulationTimeStep);
        const double d = 6.0;

//        // Compute position in Frenet coordinates
//        double s0;
//        if (previous_s_.empty())
//        {
//            s0 = ego_vehicle_data.s;
//        }
//        else
//        {
//            s0 = previous_s_.back();
//        }
//        const double s = std::fmod(s0 + s_dot_trajectory[i] * kSimulationTimeStep,
//                                   kMaxS);
//        const double d = ego_vehicle_data.d;

        // Store it for future reference
        previous_s_.push_back(s);
        previous_d_.push_back(d);

        // Output
        std::pair<double, double> xy = map.frenetToXy(s, d);

        out_x.push_back(xy.first);
        out_y.push_back(xy.second);
    }
}
