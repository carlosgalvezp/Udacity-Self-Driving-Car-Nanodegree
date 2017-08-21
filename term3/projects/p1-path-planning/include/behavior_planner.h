#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "sensor_fusion_data.h"
#include "ego_vehicle_data.h"

const int kMaxNrLanes = 3;
const double kSearchDistance = 50.0;       // [m] max distance that we search for vehicles ahead
const double kMinVehicleGapFront = 15.0;   // [m] gap forward that we must keep to other vehicles
const double kMinVehicleGapBack = 7.5;     // [m] gap forward that we must keep to other vehicles

enum class CarBehavior
{
    GO_STRAIGHT,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT,
    COMPLETE_LANE_CHANGE
};

class BehaviorPlanner
{
public:
    BehaviorPlanner();
    CarBehavior getNextAction(const EgoVehicleData& ego_vehicle,
                              const SensorFusionData& sensor_fusion);

private:
    double computeLaneCost(const EgoVehicleData& ego_vehicle,
                           const SensorFusionData& sensor_fusion,
                           const int lane_number);

    double getClosestVehicle(const EgoVehicleData& ego_vehicle,
                             const SensorFusionData& sensor_fusion,
                             const std::size_t lane_number);

    bool doing_lane_change_;
    double d_before_lane_change_;
};

#endif // BEHAVIOR_PLANNER_H
