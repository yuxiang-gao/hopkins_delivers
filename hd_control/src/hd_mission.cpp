#include "hd_control/hd_mission.h"
#include "hd_control/hd_drone_interface.h"

namespace hd_control
{
/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

void Mission::step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti, ObstacleState &ob)
{
    float speedFactor         = 2;
    float yawThresholdInDeg   = 10;

    float xCmd, yCmd, zCmd;

    geometry_msgs::Vector3 localOffset;
    localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location_);

    double xOffsetRemaining = target_offset_x_ - localOffset.x;
    double yOffsetRemaining = target_offset_y_ - localOffset.y;
    double zOffsetRemaining = target_offset_z_ - localOffset.z;

    //double yawDesiredRad     = deg2rad * target_yaw;
    double yawThresholdInRad = deg2rad * yawThresholdInDeg;
    double yawInRad          = toEulerAngle(current_atti).z;
    double yawDesiredRad      = atan2(yOffsetRemaining, xOffsetRemaining);

    ROS_INFO_THROTTLE(2, "-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x, localOffset.y, localOffset.z, yawInRad);
    ROS_INFO_THROTTLE(2, "+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining, yOffsetRemaining, zOffsetRemaining, yawInRad - yawDesiredRad);


    if (std::abs(xOffsetRemaining) >= speedFactor)
        xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
        xCmd = xOffsetRemaining;

    if (std::abs(yOffsetRemaining) >= speedFactor)
        yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
        yCmd = yOffsetRemaining;

    if (std::abs(zOffsetRemaining) >= speedFactor)
        zCmd = (zOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
    else
        zCmd = zOffsetRemaining;

    if (ob.detected)
    {
        float ob_dist = ob.distance;
        int ob_ori = ob.orientation;
        ROS_INFO_THROTTLE(1, "####### obstacle dist: %f, ori: %d ...", ob_dist, ob_ori);
        double power = obstacle_avoid_speed_ / (ob.distance + 1);
        if (ob_ori == 0)
            yCmd += power;
        else
            yCmd -= power;
        if (ob_dist <= 1)
            xCmd = 0;
    }
    //zCmd = start_gps_location_.altitude + target_offset_z_;

    /*!
    * @brief: if we already started breaking, keep break for 50 sample (1sec)
    *         and call it done, else we send normal command
    */

    if (break_counter_ > 50)
    {
        ROS_INFO("##### Route %d finished....", target_idx_);
        target_finished_ = true;
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

        if (std::abs(xOffsetRemaining) < speedFactor || std::abs(yOffsetRemaining) < speedFactor || std::abs(yawDesiredRad - yawInRad) < yawThresholdInRad)
        {
            drone_interface_ptr_->sendENUControlSignal(xCmd, yCmd, zCmd, 0, true);
        }
        else
        {

            drone_interface_ptr_->sendENUControlSignal(xCmd, yCmd, zCmd, yawDesiredRad, false);
        }
        
    }

    if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 &&
        std::abs(zOffsetRemaining) < 0.5)// &&
        //std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
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
        ROS_INFO("##### Route %d: out of bounds, reset....", target_idx_);
        inbound_counter_  = 0;
        outbound_counter_ = 0;
    }

    if (inbound_counter_ > 50)
    {
        ROS_INFO("##### Route %d start break....", target_idx_);
        break_counter_ = 1;
    }
}
}
