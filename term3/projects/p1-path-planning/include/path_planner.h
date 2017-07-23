#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

class PathPlanner
{
public:
    PathPlanner();
    void generateTrajectory(std::vector<double>& out_x,
                            std::vector<double>& out_y);
};

#endif // PATH_PLANNER_H
