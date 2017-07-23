#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>

struct VehicleData
{
    int id;     ///< ID
    double x;   ///< X position, in map coordinates
    double y;   ///< Y position, in map coordinates
    double vx;  ///< Velocity along X axis, in map coordinates
    double vy;  ///< Velocity along Y axis, in map coordinates
    double s;   ///< s Frenet coordinate
    double d;   ///< d Frenet coordinate

    VehicleData(const std::vector<double>& raw_data):
        id(raw_data[0U]),
        x(raw_data[1U]),
        y(raw_data[2U]),
        vx(raw_data[3U]),
        vy(raw_data[4U]),
        s(raw_data[5U]),
        d(raw_data[6U])
    {}
};

struct SensorFusionData
{
public:
    std::vector<VehicleData> vehicles;

    SensorFusionData(const std::vector<std::vector<double>>& raw_data):
        vehicles()
    {
        for (std::size_t i = 0U; i < raw_data.size(); ++i)
        {
            vehicles.push_back(VehicleData(raw_data[i]));
        }
    }
};

#endif // SENSOR_FUSION_H
