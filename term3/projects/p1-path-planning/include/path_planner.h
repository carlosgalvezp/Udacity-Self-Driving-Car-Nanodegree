#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "sensor_fusion_data.h"

class PathPlanner
{
public:
    PathPlanner();
    void generateTrajectory(const SensorFusionData& sensor_fusion_data,
                            std::vector<double>& out_x,
                            std::vector<double>& out_y);
};

#endif // PATH_PLANNER_H
