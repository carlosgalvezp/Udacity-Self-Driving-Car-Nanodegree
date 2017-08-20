#include "behavior_planner.h"
#include "map.h"
#include "utils.h"

#include <iostream>

BehaviorPlanner::BehaviorPlanner():
    doing_lane_change_(false),
    d_before_lane_change_(0.0)
{
}

CarBehavior BehaviorPlanner::getNextAction(const EgoVehicleData& ego_vehicle,
                                           const SensorFusionData& sensor_fusion)
{
    CarBehavior output;

    if (doing_lane_change_)
    {
        std::cout << "DOING LANE CHANGE " << ego_vehicle.d << "," << d_before_lane_change_ << std::endl;
        output = CarBehavior::COMPLETE_LANE_CHANGE;

        // Check if lane change complete
        const double d_diff = std::abs(ego_vehicle.d - d_before_lane_change_);
        if (std::abs(d_diff - 4.0) < 0.1)
        {
            doing_lane_change_ = false;
            std::cout << "STOPPED LANE CHANGE" << std::endl;
        }
    }
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
        std::cout << "[" << ego_lane << "] ";
        for(int lane : available_lanes)
        {
            const double cost = computeLaneCost(ego_vehicle, sensor_fusion, lane);
            std::cout << cost << ",";
            if (cost < min_cost)
            {
                min_cost = cost;
                best_lane = lane;
            }
        }

        std::cout << std::endl;

        // Output desired behaviour
        if (best_lane == (ego_lane - 1))
        {
            output = CarBehavior::CHANGE_LANE_LEFT;
            doing_lane_change_ = true;
            d_before_lane_change_ = ego_vehicle.d;

            std::cout << "CHANGE_LEFT!" << std::endl;
        }
        else if (best_lane == (ego_lane + 1))
        {
            output = CarBehavior::CHANGE_LANE_RIGHT;
            doing_lane_change_ = true;
            d_before_lane_change_ = ego_vehicle.d;

            std::cout << "CHANGE_RIGHT!" << std::endl;
        }
        else
        {
            output = CarBehavior::GO_STRAIGHT;
        }
    }

    return output;
}

double BehaviorPlanner::computeLaneCost(const EgoVehicleData& ego_vehicle,
                                        const SensorFusionData& sensor_fusion,
                                        const int lane_number)
{
    double gap_vehicle_front = kSearchDistance;
    double gap_vehicle_back = kSearchDistance;
    double lane_velocity = mph2ms(60.0);

    const int ego_lane = Map::getLaneNumber(ego_vehicle.d);

    for (const VehicleData& vehicle : sensor_fusion.vehicles)
    {
        const int vehicle_lane = Map::getLaneNumber(vehicle.d);

        if (vehicle_lane == lane_number)
        {
            double gap = Map::s_diff(vehicle.s, ego_vehicle.s);

            if (gap > 0.0)  // In front of us
            {
                const double gap = Map::s_diff(vehicle.s, ego_vehicle.s);
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
            else                            // Behind
            {
                gap = -gap;
                if (gap < kSearchDistance && gap < gap_vehicle_back)
                {
                    gap_vehicle_back = gap;
                }
            }
        }
    }

    double goodness = 0.2 * (gap_vehicle_front / kSearchDistance) +
                      0.7 * (lane_velocity / mph2ms(60.0)) +
                      0.05 * (lane_number == 1);

    if (ego_lane != lane_number)
    {
        if (gap_vehicle_back  < kMinVehicleGap ||
            gap_vehicle_front < kMinVehicleGap)
        {
            // Infinite cost if we try to change lane but there's no gap
            goodness = std::numeric_limits<double>::min();
        }
    }

    return 1.0 / goodness;
}
