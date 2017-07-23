#include "path_planner.h"
#include <iostream>

PathPlanner::PathPlanner()
{

}

void PathPlanner::generateTrajectory(const SensorFusionData& sensor_fusion_data,
                                     std::vector<double> &out_x,
                                     std::vector<double> &out_y)
{
    for(const VehicleData& vehicle : sensor_fusion_data.vehicles)
    {
        std::cout << "[" << vehicle.id << "] v = ("
                  << vehicle.vx << ","
                  << vehicle.vy << ")" << std::endl;
    }
}
