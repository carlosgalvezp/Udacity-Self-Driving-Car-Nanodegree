#include "behavior_planner.h"

BehaviorPlanner::BehaviorPlanner()
{
}

CarBehavior BehaviorPlanner::getNextAction()
{
    return CarBehavior::GO_STRAIGHT;
}
