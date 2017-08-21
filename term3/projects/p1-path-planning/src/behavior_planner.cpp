#include "behavior_planner.h"

#include <iostream>

#include "map.h"
#include "utils.h"

BehaviorPlanner::BehaviorPlanner():
    doing_lane_change_(false),
    d_before_lane_change_(0.0)
{
}

CarBehavior BehaviorPlanner::getNextAction(const EgoVehicleData& ego_vehicle,
                                           const SensorFusionData& sensor_fusion)
{
    CarBehavior output;

    // If we are doing a lane change, wait until completion
    if (doing_lane_change_)
    {
        std::cout << "COMPLETE_LANE_CHANGE" << std::endl;
        output = CarBehavior::COMPLETE_LANE_CHANGE;

        // Check if lane change complete
        const double d_diff = std::abs(ego_vehicle.d - d_before_lane_change_);
        if (std::abs(d_diff - kLaneWidth) < 0.2)
        {
            doing_lane_change_ = false;
        }
    }
    // Otherwise, decide what action to take next
    else
    {
        // Get our lane
        const int ego_lane = Map::getLaneNumber(ego_vehicle.d);

        // Get available lanes to go
        std::vector<int> available_lanes;
        available_lanes.push_back(ego_lane);
        if (ego_lane - 1 >= 0)          { available_lanes.push_back(ego_lane - 1); }
        if (ego_lane + 1 < kMaxNrLanes) { available_lanes.push_back(ego_lane + 1); }

        // Choose the one with lowest cost
        int best_lane = 0;
        double min_cost = std::numeric_limits<double>::max();
        for(int lane : available_lanes)
        {
            const double cost = computeLaneCost(ego_vehicle, sensor_fusion, lane);
            if (cost < min_cost)
            {
                min_cost = cost;
                best_lane = lane;
            }
        }

        // Output optimal behaviour
        if (best_lane == (ego_lane - 1))
        {
            std::cout << "CHANGE LEFT" << std::endl;

            output = CarBehavior::CHANGE_LANE_LEFT;
            doing_lane_change_ = true;
            d_before_lane_change_ = ego_vehicle.d;
        }
        else if (best_lane == (ego_lane + 1))
        {
            std::cout << "CHANGE RIGHT" << std::endl;

            output = CarBehavior::CHANGE_LANE_RIGHT;
            doing_lane_change_ = true;
            d_before_lane_change_ = ego_vehicle.d;
        }
        else
        {
            std::cout << "GO STRAIGHT" << std::endl;

            output = CarBehavior::GO_STRAIGHT;
        }
    }

    return output;
}

double BehaviorPlanner::computeLaneCost(const EgoVehicleData& ego_vehicle,
                                        const SensorFusionData& sensor_fusion,
                                        const int lane_number)
{
    // Variables to keep the gap w.r.t to vehicles front and back.
    // Initialize to "kSearchDistance" so we don't consider vehicles
    // farther away from that
    double gap_vehicle_front = kSearchDistance;
    double gap_vehicle_back = kSearchDistance;

    // Speed at which the closest vehicle is moving in this lane
    double lane_velocity = kRoadSpeedLimit;

    const int ego_lane = Map::getLaneNumber(ego_vehicle.d);

    // Loop over vehicles
    for (const VehicleData& vehicle : sensor_fusion.vehicles)
    {
        const int vehicle_lane = Map::getLaneNumber(vehicle.d);

        // Consider only vehicles in lane under study
        if (vehicle_lane == lane_number)
        {
            // Get the distance (in s-coordinates) from ego
            double gap = Map::s_min_diff(vehicle.s, ego_vehicle.s);

            if (gap > 0.0)  // In front of us
            {
                // Update gap_vehicle_front in case this vehicle is closer to us.
                // Also keep track of its velocity.
                if (gap < gap_vehicle_front)
                {
                    gap_vehicle_front = gap;

                    const double v = std::sqrt(vehicle.vx * vehicle.vx +
                                               vehicle.vy * vehicle.vy);
                    if (v < lane_velocity)
                    {
                        lane_velocity = v;
                    }
                }
            }
            else            // Behind
            {
                gap = -gap;
                if (gap < gap_vehicle_back)
                {
                    gap_vehicle_back = gap;
                }
            }
        }
    }

    // Compute the score for this lane
    double score = 0.2 * (gap_vehicle_front / kSearchDistance) +  // vehicles far away
                   0.7 * (lane_velocity / kRoadSpeedLimit) +      // high lane speed
                   0.05 * (lane_number == 1);   // center lane gives more lane-change options

    // Infinite cost if we try to change lane but there's no gap for us
    // to move into
    if (ego_lane != lane_number)
    {
        if (gap_vehicle_back  < kMinVehicleGapBack ||
            gap_vehicle_front < kMinVehicleGapFront)
        {
            score = std::numeric_limits<double>::min();
        }
    }

    // Return the cost as the inverse of the score
    return 1.0 / score;
}
