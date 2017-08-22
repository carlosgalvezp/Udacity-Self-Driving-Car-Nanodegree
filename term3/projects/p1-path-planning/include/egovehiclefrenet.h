#ifndef EGOVEHICLEFRENET_H
#define EGOVEHICLEFRENET_H

/// Struct to represent the ego-vehicle state in Frenet coordinates
struct EgoVehicleFrenet
{
    double s;
    double s_dot;
    double s_ddot;

    double d;
    double d_dot;
    double d_ddot;
};

#endif // EGOVEHICLEFRENET_H
