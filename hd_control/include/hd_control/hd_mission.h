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

typedef strcut FlightTarget
{
    float x = 0;
    float y = 0;
    float z = 0;
    float yaw = 0;
} FlightTarget;

typedef struct ObstacleState
{
    bool  detected = false;
    float distace = 0.0;
    int orientation = 0; //0 for left 1 for right
} ObstacleState;

class Mission
{
public:
    // The basic state transition flow is:
    // 0---> 1 ---> 2 ---> ... ---> N ---> 0
    // where state 0 means the mission is note started
    // and each state i is for the process of moving to a target point.
    int state_;
    bool finished_;
    float working_height_; // for test only

    Mission(DroneInterfacePtr drone_interface_ptr, sensor_msgs::NavSatFix &current_gps, geometry_msgs::Point &current_local_pos) : 
        state_(0), 
        finished_(false),
        working_height_(5.0)
        inbound_counter_(0), 
        outbound_counter_(0), 
        break_counter_(0),
        target_offset_x_(0.0), 
        target_offset_y_(0.0), 
        target_offset_z_(0.0),
        drone_interface_ptr_(drone_interface_ptr)
        start_gps_location_(current_gps),
        start_local_position_(current_local_pos)
    {
        FlightPlan fp;
        fp.z = working_height_;
        flight_plan_.clear();
        flight_plan_.append(fp);
    }

    void step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti, ObstacleState &ob);

    void setPlan(std::vector<FlightTarget> flight_targets)
    {
        state_ = 0;
        flight_plan_ = flight_targets;
    }

    void setGPSPlan(std::vector<sensor_msgs::NavSatFix> flight_targets)
    {
        state_ = 0;
        gps_flight_plan_ = flight_targets;
        
    }

    void appendPlan(FlightTarget flight_target)
    {
        gps_flight_plan_.append(flight_target);
    }

    void uploadPlan()
    {
        FlightPlan fp;
        fp.z = working_height_;
        flight_plan_.clear();
        flight_plan_.append(fp);
    }

    void clearPlan()
    {
        state_ = 0;
        gps_flight_plan_.clear();
        
    }

    void setTarget(FlightTarget ft)
    {
        target_offset_x_ = ft.x;
        target_offset_y_ = ft.y;
        target_offset_z_ = ft.z;
        target_yaw_      = ft.yaw;
    }

    void setTarget(float x, float y, float z, float yaw)
    {
        target_offset_x_ = x;
        target_offset_y_ = y;
        target_offset_z_ = z;
        target_yaw_      = yaw;
    }

    void setTarget(sensor_msgs::NavSatFix &target)
    {
        eometry_msgs::Vector3  target_offset;
        localOffsetFromGpsOffset(target_offset, target, start_gps_location_);
        target_offset_x_ = target_offset.x;
        target_offset_y_ = target_offset.y;
        target_offset_z_ = target_offset.z;
        target_yaw_      = yaw;
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

    
    void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                                    sensor_msgs::NavSatFix& target,
                                    sensor_msgs::NavSatFix& origin)
    {
        double deltaLon = target.longitude - origin.longitude;
        double deltaLat = target.latitude - origin.latitude;

        deltaNed.y = deltaLat * deg2rad * C_EARTH;
        deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
        deltaNed.z = target.altitude - origin.altitude;
    }

    geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
    {
        geometry_msgs::Vector3 ans;

        tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
        R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
        return ans;
    }

private:
    std::vector<FlightTarget> flight_plan_;
    std::vector<sensor_msgs::NavSatFix> gps_flight_plan_;

    int inbound_counter_;
    int outbound_counter_;
    int break_counter_;

    float target_offset_x_;
    float target_offset_y_;
    float target_offset_z_;
    float target_yaw_;

    sensor_msgs::NavSatFix start_gps_location_;
    geometry_msgs::Point start_local_position_;

    DroneInterfacePtr drone_interface_ptr_;
};

typedef boost::shared_ptr<Mission> MissionPtr;
} // namespace hd_control
#endif // HD_MISSION_H