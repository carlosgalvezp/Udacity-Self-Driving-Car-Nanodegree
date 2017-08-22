#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "sensor_fusion_data.h"
#include "ego_vehicle_data.h"

/// Max number of drivable lanes
const int kMaxNrLanes = 3;

/// Max distance to search for vehicles in front or behind of us
const double kSearchDistance = 50.0;       // [m]

/// Minimum gap to closest vehicle in front of us, before attempting to change lane
const double kMinVehicleGapFront = 15.0;   // [m]

/// Minimum gap to closest vehicle behind us, before attempting to change lane
const double kMinVehicleGapBack = 7.5;     // [m]

/// \brief Enum defining which behaviour the car should follow
enum class CarBehavior
{
    GO_STRAIGHT,          ///< Continue in the same lane, adjusting speed as necessary
    CHANGE_LANE_LEFT,     ///< Initiate change left
    CHANGE_LANE_RIGHT,    ///< Initiate change right
    COMPLETE_LANE_CHANGE  ///< Continue lane change maneuver until finished
};

/// Class responsible for the Behavior Planner module, in charge of
/// deciding the best possible action for the vehicle to move forward.
class BehaviorPlanner
{
public:
    BehaviorPlanner();

    /// \brief Determines the next action to take
    /// \param ego_vehicle ego-vehicle data
    /// \param sensor_fusion sensor fusion data
    /// \return the next action to take
    CarBehavior getNextAction(const EgoVehicleData& ego_vehicle,
                              const SensorFusionData& sensor_fusion);

private:
    double computeLaneCost(const EgoVehicleData& ego_vehicle,
                           const SensorFusionData& sensor_fusion,
                           const int lane_number);

    double getClosestVehicle(const EgoVehicleData& ego_vehicle,
                             const SensorFusionData& sensor_fusion,
                             const std::size_t lane_number);

    bool doing_lane_change_;
    double target_d_;
};

#endif // BEHAVIOR_PLANNER_H
