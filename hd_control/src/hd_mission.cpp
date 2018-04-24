#include "hd_control/hd_mission.h"
#include "hd_control/hd_drone_interface.h"

namespace hd_control
{
/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

void Mission::step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti, ObstacleState &ob)
{
    // add input ob_dist, ob_ori
    float ob_dist = ob.distance;
    int ob_ori = ob.orientation;
    // if ( ob_dist == 0 )
    // {

    // }
    // else if ( ob_dist >= 5)
    // {
    //     // add y speed
    // }
    // else if ( ob_dist > 1 && ob_dist < 5)
    // {
    //     // add greater y speed
    // }
    // else if (ob_dist < 1)
    // {
    //     // stop and move sideways
    // }
    geometry_msgs::Vector3 localOffset;

    float speedFactor         = 2;
    float yawThresholdInDeg   = 2;

    float xCmd, yCmd, zCmd;

    localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location_);

    double xOffsetRemaining = target_offset_x_ - localOffset.x;
    double yOffsetRemaining = target_offset_y_ - localOffset.y;
    double zOffsetRemaining = target_offset_z_ - localOffset.z;



    ROS_INFO_THROTTLE(2, "-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x, localOffset.y, localOffset.z, yawInRad);
    ROS_INFO_THROTTLE(2, "+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining, yOffsetRemaining, zOffsetRemaining, yawInRad - yawDesiredRad);

    if (std::abs(xOffsetRemaining) >= 5 * speedFactor || std::abs(yOffsetRemaining) >= 5 * speedFactor)
    {
        //TODO
    }
    else
    {
        double yawDesiredRad     = deg2rad * target_yaw;
        double yawThresholdInRad = deg2rad * yawThresholdInDeg;
        double yawInRad          = toEulerAngle(current_atti).z;
    }
    
    if (std::abs(xOffsetRemaining) >= speedFactor)
        xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
        xCmd = xOffsetRemaining;

    if (std::abs(yOffsetRemaining) >= speedFactor)
        yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
        yCmd = yOffsetRemaining;

    zCmd = start_gps_location_.altitude + target_offset_z_;

    /*!
    * @brief: if we already started breaking, keep break for 50 sample (1sec)
    *         and call it done, else we send normal command
    */

    if (break_counter_ > 50)
    {
        ROS_INFO("##### Route %d finished....", state);
        finished = true;
        return;
    }
    else if(break_counter_ > 0)
    {
        drone_interface_ptr_->sendControlSignal(0, 0, 0, 0);
        break_counter_++;
        return;
    }
    else //break_counter = 0, not in break stage
    {
        drone_interface_ptr_->sendENUControlSignal(xCmd, yCmd, zCmd, yawDesiredRad, False);
    }

    if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 &&
        std::abs(zOffsetRemaining) < 0.5 &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
        //! 1. We are within bounds; start incrementing our in-bound counter
        inbound_counter_ ++;
    }
    else
    {
        if (inbound_counter_ != 0)
        {
            //! 2. Start incrementing an out-of-bounds counter
            outbound_counter_ ++;
        }
    }

    //! 3. Reset withinBoundsCounter if necessary
    if (outbound_counter_ > 10)
    {
        ROS_INFO("##### Route %d: out of bounds, reset....", state);
        inbound_counter_  = 0;
        outbound_counter_ = 0;
    }

    if (inbound_counter_ > 50)
    {
        ROS_INFO("##### Route %d start break....", state);
        break_counter_ = 1;
    }
}
}