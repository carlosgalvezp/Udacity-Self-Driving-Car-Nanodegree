#include "behavior_planner.h"
#include "map.h"
#include "utils.h"

#include <iostream>

BehaviorPlanner::BehaviorPlanner()
{
}

CarBehavior BehaviorPlanner::getNextAction(const EgoVehicleData& ego_vehicle,
                                           const SensorFusionData& sensor_fusion)
{
    // Compute velocity for each lane
    const double road_speed_limit = mph2ms(49.5); // TODO
    const double safety_distance = 30.0;  // [m]

    std::vector<double> lane_velocities(3U, road_speed_limit);

    for (const VehicleData& vehicle : sensor_fusion.vehicles)
    {
        const double s_vehicle = vehicle.s;
        const double d_vehicle = vehicle.d;

        const double s_ego = ego_vehicle.s;

        // Consider only the ones in our direction and in front of us
        if (d_vehicle > 0.0                            // in our direction
            && s_vehicle > s_ego                       // in front of us
            && (s_vehicle - s_ego) < safety_distance)  // too close
        {
            // Compute velocity and distance to ego
            const double v_vehicle = std::sqrt(vehicle.vx * vehicle.vx + vehicle.vy + vehicle.vy);

            const int lane_nr = Map::getLaneNumber(d_vehicle);

            if (lane_nr >= 0 && lane_nr < static_cast<int>(lane_velocities.size()))
            {
                lane_velocities[lane_nr] = std::min(lane_velocities[lane_nr], v_vehicle);
            }
        }
    }

    // Get current lane
    const double ego_lane = Map::getLaneNumber(ego_vehicle.d);

    // Decide if it's required to continue straight or change lane
    CarBehavior output;

    if (std::abs(lane_velocities[ego_lane] - road_speed_limit) < 0.1)
    {
        output = CarBehavior::GO_STRAIGHT;
    }
    else
    {
        const double left_lane = ego_lane - 1;
        const double right_lane = ego_lane + 1;

        const double v_left = left_lane >= 0 ? lane_velocities[left_lane] : 0.0;
        const double v_right = right_lane < lane_velocities.size() ? lane_velocities[right_lane] : 0.0;

        if (v_left > v_right)
        {
            output = CarBehavior::CHANGE_LANE_LEFT;
            std::cout << "CHANGE LEFT!";
        }
        else
        {
            output = CarBehavior::CHANGE_LANE_RIGHT;
            std::cout << "CHANGE RIGHT!";
        }

        std::cout << " - VL: " << v_left << " VR: " << v_right
                  << " - Ego lane: " << ego_lane << std::endl;
    }

    return output;
}
