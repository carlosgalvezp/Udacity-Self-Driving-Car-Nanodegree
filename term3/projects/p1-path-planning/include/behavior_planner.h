#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

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
    CarBehavior getNextAction();
};

#endif // BEHAVIOR_PLANNER_H
