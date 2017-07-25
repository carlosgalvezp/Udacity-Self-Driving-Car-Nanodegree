#ifndef EGO_VEHICLE_DATA_H
#define EGO_VEHICLE_DATA_H

struct EgoVehicleData
{
    double x;       ///< X position in global coordinates
    double y;       ///< Y position in global coordinates
    double s;       ///< s Frenet coordinate w.r.t. the road
    double d;       ///< d Frenet coordinate w.r.t. the road
    double yaw;     ///< yaw of the vehicle, in global coordinates
    double speed;   ///< forward speed of the vehicle
};

#endif // EGO_VEHICLE_DATA_H
