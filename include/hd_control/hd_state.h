#ifndef HD_STATE_H
#define HD_STATE_H

#include <boost/shared_ptr.hpp>

namespace  hd_control
{
struct StateConst
{
    enum class DroneState
    {
        STATE_ON_GROUND = 0;
        STATE_IN_AIR = 1;
        STATE_WAYPOINT_MISSION = 2;
        STATE_OBSTACLE = 3;
        STATE_WAYPOINT_GOAL = 4;
        STATE_SEACHING = 5;
        STATE_ALIGNING = 6;
        STATE_DESCENDING = 7;
        STATE_ON_GOAL = 8;
    }; // enum drone_state_t

    enum class PackageState
    {
        PACKAGE_OFF = 0;
        PACKAGE_ON = 1;
    }; // enum package_state_t

    typedef boost::shared_ptr<DroneState> DroneStatePtr;
    typedef boost::shared_ptr<PackageState> PackageStatePtr;
    typedef std::pair<DroneState, PackageState> StatePair;
}; // struct StateNames
} // namespace  hd_state_machine

#endif