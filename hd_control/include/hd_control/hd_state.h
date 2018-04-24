#ifndef HD_STATE_H
#define HD_STATE_H

#include <boost/shared_ptr.hpp>

namespace  hd_control
{
enum class DroneState
{
    STATE_ON_GROUND = 0, // on ground
    STATE_IN_AIR = 1,    // taken off
    STATE_IN_MISSION = 2, //start waypoint mission
    STATE_OBSTACLE = 3,   //obstacle
    STATE_NEAR_GOAL = 4,  //within certain bound to the gps goal
    STATE_SEARCHING = 5,  // seraching for tag
    STATE_ALIGNING = 6,  //position tracking
    STATE_DESCENDING = 7, //landing
    STATE_ON_GOAL = 8     //landed
}; // enum class DroneState

typedef struct HDStates
{
    bool package_state;
    DroneState drone_state;
    HDStates(DroneState d, bool p): drone_state(d), package_state(p)
    {}
    HDStates()
    {}

} HDStates;
} // namespace  hd_control

#endif
