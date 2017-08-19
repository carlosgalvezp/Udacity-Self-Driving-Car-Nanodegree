#ifndef VEHICLE_H
#define VEHICLE_H

/// \brief Class representing a vehicle in the road
class Vehicle
{
public:
    Vehicle();

    double s;
    double d;
    double x;
    double y;
    double speed;
    double yaw;

    /// \brief Predicts the state of the vehicle at time t
    /// \param t time where the state of the vehicle is predicted
    /// \return the new state of the vehicle
    Vehicle predictTo(double t);
};

#endif // VEHICLE_H
