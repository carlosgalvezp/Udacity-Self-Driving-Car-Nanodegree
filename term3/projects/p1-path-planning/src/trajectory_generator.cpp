#include "trajectory_generator.h"
#include <iostream>

#include "utils.h"

TrajectoryGenerator::TrajectoryGenerator()
{
}

void TrajectoryGenerator::generateTrajectory(const CarBehavior next_action,
                                             const EgoVehicleData& ego_vehicle_data,
                                             const MapData& map_data,
                                             const std::vector<double>& previous_x,
                                             const std::vector<double>& previous_y,
                                             std::vector<double>& out_x,
                                             std::vector<double>& out_y)
{
    switch (next_action)
    {
        case CarBehavior::GO_STRAIGHT:
            generateTrajectoryFollowLane(ego_vehicle_data, map_data, previous_x, previous_y, out_x, out_y);
            break;
        default:
            break;
    }
}

void TrajectoryGenerator::generateTrajectoryFollowLane(const EgoVehicleData& ego_vehicle_data,
                                                       const MapData& map_data,
                                                       const std::vector<double>& previous_x,
                                                       const std::vector<double>& previous_y,
                                                       std::vector<double>& out_x,
                                                       std::vector<double>& out_y)
{
    // Set final time
//    const std::size_t N_KEEP = 10U;
    const std::size_t path_size = 0;//std::min(N_KEEP, previous_x.size());

    const double t = (kNrTrajectoryPoints - path_size) * kSimulationTimeStep;

    // Set initial state
    double s0, s0_d;

    if (path_size > 1U)
    {
        const double prev_x  = previous_x[path_size - 1U];
        const double prev_x2 = previous_x[path_size - 2U];

        const double prev_y  = previous_y[path_size - 1U];
        const double prev_y2 = previous_y[path_size - 2U];


        const double prev_angle = std::atan2(prev_y - prev_y2,
                                             prev_x - prev_x2);

        s0 = getFrenet(prev_x, prev_y, prev_angle, map_data.x, map_data.y)[0U];
        s0_d = std::sqrt((prev_x - prev_x2)*(prev_x - prev_x2) +
                         (prev_y - prev_y2)*(prev_y - prev_y2)) / kSimulationTimeStep;
    }
    else
    {
        s0    = ego_vehicle_data.s;
        s0_d  = ego_vehicle_data.speed;
    }

    const double s0_dd = 0.0;

    // Set final state
    const double sf    = 10.0;
    const double sf_d  = 10.0;
    const double sf_dd = 0.0;

    // Compute Jerk-minimizing trajectory
    std::vector<double> trajectory_coeffs;
    generateJerkMinTrajectory(s0, s0_d, s0_dd, sf, sf_d, sf_dd, t, trajectory_coeffs);

    // Output
    out_x.clear();
    out_y.clear();

//    for (std::size_t i = 0U; i < path_size; ++i)
//    {
//        out_x.push_back(previous_x[i]);
//        out_y.push_back(previous_y[i]);
//    }

    for (std::size_t i = 0U; i < kNrTrajectoryPoints - path_size; ++i)
    {
//        const double t_i = kSimulationTimeStep * static_cast<double>(i+1);
        const double s_next = s0 + 1*i;//evaluatePolynomial(trajectory_coeffs, t_i);
        const double d_next = ego_vehicle_data.d;

        const std::vector<double> xy = getXY(s_next, d_next, map_data.s, map_data.x, map_data.y);

        out_x.push_back(xy[0U]);
        out_y.push_back(xy[1U]);
    }
}
