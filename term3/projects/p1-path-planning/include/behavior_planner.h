#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "sensor_fusion_data.h"
#include "ego_vehicle_data.h"

enum class CarBehavior
{
    GO_STRAIGHT,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT
};

class BehaviorPlanner
{
public:
    BehaviorPlanner();
    CarBehavior getNextAction(const EgoVehicleData& ego_vehicle,
                              const SensorFusionData& sensor_fusion);
};

#endif // BEHAVIOR_PLANNER_H
