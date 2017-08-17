#include "behavior_planner.h"

BehaviorPlanner::BehaviorPlanner()
{
}

CarBehavior BehaviorPlanner::getNextAction(const EgoVehicleData& ego_vehicle,
                                           const SensorFusionData& sensor_fusion)
{
    // Get distance to next car

    // If big enough, continue in lane

    // Otherwise, evalute if lane change is required
    // Get the current lane

    // Get the velocity all the lanes

    // If current lane is okey, go straight

    // Otherwise, go to the fastest adjacent lane
    return CarBehavior::GO_STRAIGHT;
}
