#include "trajectory_generator.h"

#include "utils.h"

TrajectoryGenerator::TrajectoryGenerator()
{
}

void TrajectoryGenerator::generateTrajectory(const CarBehavior next_action,
                                             const EgoVehicleData& ego_vehicle_data,
                                             const MapData& map_data,
                                             std::vector<double>& out_x,
                                             std::vector<double>& out_y)
{
    switch (next_action)
    {
        case CarBehavior::GO_STRAIGHT:
            generateTrajectoryFollowLane(ego_vehicle_data, map_data, out_x, out_y);
            break;
        default:
            break;
    }
}

void TrajectoryGenerator::generateTrajectoryFollowLane(const EgoVehicleData& ego_vehicle_data,
                                                       const MapData& map_data,
                                                       std::vector<double>& out_x,
                                                       std::vector<double>& out_y)
{
    // Set initial state
    const double s0 = ego_vehicle_data.s;
    const double s0_d = ego_vehicle_data.speed;
    const double s0_dd = 0.0;

    // Set final state
    const double sf = s0 + 100.0;
    const double sf_d = 22.352;  // 50 mph
    const double sf_dd = 0.0;

    // Set final time
    const double t = kNrTrajectoryPoints * kSimulationTimeStep;

    // Compute Jerk-minimizing trajectory
    std::vector<double> trajectory_coeffs;
    generateJerkMinTrajectory(s0, s0_d, s0_dd, sf, sf_d, sf_dd, t, trajectory_coeffs);

    // Output
    out_x.resize(kNrTrajectoryPoints);
    out_y.resize(kNrTrajectoryPoints);

    for (std::size_t i = 0U; i < out_x.size(); ++i)
    {
        const double t_i = kSimulationTimeStep * i;
        const double s_next = evaluatePolynomial(trajectory_coeffs, t_i);
        const double d_next = 0.0;

        const std::vector<double> xy = getXY(s_next, d_next, map_data.s, map_data.x, map_data.y);

        out_x[i] = xy[0U];
        out_y[i] = xy[1U];
    }
}
