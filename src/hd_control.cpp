#include "hd_control/hd_control.h"

namespace hd_control
{
DroneControl::DroneControl(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
    drone_interface_ptr_.reset(new DroneInterface(nh, nh_priv));

    velocity_control_x_sub_ = nh.subscribe("/hd/position_track/velocity_control_effort_x", 10, &velocityControlEffortXCallback);
    velocity_control_y_sub_ = nh.subscribe("/hd/position_track/velocity_control_effort_y", 10, &velocityControlEffortYCallback);
    velocity_control_yaw_sub_ = nh.subscribe("/hd/position_track/velocity_control_effort_yaw", 10, &velocityControlEffortYawCallback);
    position_track_enable_sub_ = nh.subscribe("/hd/position_track/position_track_enable", 1, &positionTrackEnableCallback);
    landing_condition_met_sub_ = nh.subscribe("/hd/position_track/landing_condition_met", 1, &landingConditionMetCallback);
    relanding_condition_met_sub_ = nh.subscribe("/hd/position_track/relanding_condition_met", 1, &relandingConditionMetCallback);
    obstacle_detection_sub_ = nh.subscribe("/hd/obstacle_avoidance/obstacles", 10, &obstcleCallback);

    while(ros::ok())
    {
        ros::spinOnce();
        if (position_track_enabled_)
        {
            if (landing_condition_met_)
            {
                sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, descending_speed_, velocity_control_effort_yaw_);
            }
            else if (relanding_condition_met_)
            {
                sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, ascending_speed_, velocity_control_effort_yaw_);
            }
            else
            {
                sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, 0.0, velocity_control_effort_yaw_);
            }
        }
        else continue;
    }

}
}