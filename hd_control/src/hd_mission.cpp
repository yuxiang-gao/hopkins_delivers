#include "hd_control/hd_mission.h"
#include "hd_control/hd_drone_interface.h"

namespace hd_control
{
/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
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

void Mission::step(sensor_msgs::NavSatFix &current_gps, geometry_msgs::Quaternion &current_atti)
{
    geometry_msgs::Vector3 localOffset;

    float speedFactor         = 2;
    float yawThresholdInDeg   = 2;

    float xCmd, yCmd, zCmd;

    localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location_);

    double xOffsetRemaining = target_offset_x - localOffset.x;
    double yOffsetRemaining = target_offset_y - localOffset.y;
    double zOffsetRemaining = target_offset_z - localOffset.z;

    double yawDesiredRad     = deg2rad * target_yaw;
    double yawThresholdInRad = deg2rad * yawThresholdInDeg;
    double yawInRad          = toEulerAngle(current_atti).z;

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

    zCmd = start_gps_location_.altitude + target_offset_z;

    /*!
    * @brief: if we already started breaking, keep break for 50 sample (1sec)
    *         and call it done, else we send normal command
    */

    if (break_counter > 50)
    {
        ROS_INFO("##### Route %d finished....", state);
        finished = true;
        return;
    }
    else if(break_counter > 0)
    {
        drone_interface_ptr_->sendControlSignal(0, 0, 0, 0);
        break_counter++;
        return;
    }
    else //break_counter = 0, not in break stage
    {
        sensor_msgs::Joy controlPosYaw;


        controlPosYaw.axes.push_back(xCmd);
        controlPosYaw.axes.push_back(yCmd);
        controlPosYaw.axes.push_back(zCmd);
        controlPosYaw.axes.push_back(yawDesiredRad);
        ctrlPosYawPub.publish(controlPosYaw);
    }

    if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 &&
        std::abs(zOffsetRemaining) < 0.5 &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
        //! 1. We are within bounds; start incrementing our in-bound counter
        inbound_counter ++;
    }
    else
    {
        if (inbound_counter != 0)
        {
        //! 2. Start incrementing an out-of-bounds counter
        outbound_counter ++;
        }
    }

    //! 3. Reset withinBoundsCounter if necessary
    if (outbound_counter > 10)
    {
        ROS_INFO("##### Route %d: out of bounds, reset....", state);
        inbound_counter  = 0;
        outbound_counter = 0;
    }

    if (inbound_counter > 50)
    {
        ROS_INFO("##### Route %d start break....", state);
        break_counter = 1;
    }
}
}