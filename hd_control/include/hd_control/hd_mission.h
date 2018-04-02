#ifndef HD_MISSION_H
#define HD_MISSION_H

#include <cmath>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

#include "hd_control/hd_state.h"

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#include <boost/shared_ptr.hpp>

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
namespace hd_control
{
#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

typedef boost::shared_ptr<Mission> MissionPtr;

typedef strcut FlightTarget
{
    float x = 0;
    float y = 0;
    float z = 0;
    float yaw = 0;
} FlightTarget;

class Mission
{
public:
    // The basic state transition flow is:
    // 0---> 1 ---> 2 ---> ... ---> N ---> 0
    // where state 0 means the mission is note started
    // and each state i is for the process of moving to a target point.
    int state;
    std::vector<FlightTarget> flight_plan_;
    std::vector<sensor_msgs::NavSatFix> gps_flight_plan_;

    int inbound_counter;
    int outbound_counter;
    int break_counter;

    float target_offset_x;
    float target_offset_y;
    float target_offset_z;
    float target_yaw;

    sensor_msgs::NavSatFix start_gps_location_;
    geometry_msgs::Point start_local_position_;

    DroneInterfacePtr drone_interface_ptr_;

    bool finished;

    Mission(DroneInterfacePtr drone_interface_ptr, sensor_msgs::NavSatFix &current_gps, geometry_msgs::Point &current_local_pos) : 
        state(0), 
        inbound_counter(0), 
        outbound_counter(0), 
        break_counter(0),
        target_offset_x(0.0), 
        target_offset_y(0.0), 
        target_offset_z(0.0),
        finished(false), 
        drone_interface_ptr_(drone_interface_ptr)
        start_gps_location_(current_gps),
        start_local_position_(current_local_pos)
    {
    }

    void step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti);

    void setPlan(std::vector<FlightTarget> flight_targets)
    {
        state = 0;
        flight_plan_ = flight_targets;
    }

    void setPlan(std::vector<sensor_msgs::NavSatFix> flight_targets)
    {
        state = 0;
        gps_flight_plan_ = flight_targets;
    }

    void setTarget(FlightTarget ft)
    {
        target_offset_x = ft.x;
        target_offset_y = ft.y;
        target_offset_z = ft.z;
        target_yaw      = ft.yaw;
    }

    void setTarget(float x, float y, float z, float yaw)
    {
        target_offset_x = x;
        target_offset_y = y;
        target_offset_z = z;
        target_yaw      = yaw;
    }

    void setTarget(sensor_msgs::NavSatFix &target)
    {
        eometry_msgs::Vector3  target_offset;
        localOffsetFromGpsOffset(target_offset, target, start_gps_location_);
        target_offset_x = target_offset.x;
        target_offset_y = target_offset.y;
        target_offset_z = target_offset.z;
        target_yaw      = yaw;
    }

    void reset(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Point &current_local_pos)
    {
        inbound_counter = 0;
        outbound_counter = 0;
        break_counter = 0;
        finished = false;
        start_gps_location_ = current_gps;
        start_local_position_ = current_local_pos;
    }
};

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                              sensor_msgs::NavSatFix& target,
                              sensor_msgs::NavSatFix& origin);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

} // namespace hd_control
#endif // DEMO_FLIGHT_CONTROL_H