#ifndef HD_CONTROL_H
#define HD_CONTROL_H


#include "hd_control/hd_drone_interface.h"

namespace hd_control
{
class DroneControl
{
public:
    DroneControl(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
private:
    ~DroneControl();    
    // ros::NodeHandle nh_;
    // ros::NodeHandle nh_priv_("~");
    DroneInterfacePtr drone_interface_ptr_;

    ros::Subscriber velocity_control_x_sub_;
    ros::Subscriber velocity_control_y_sub_;
    ros::Subscriber velocity_control_yaw_sub_;

    ros::Subscriber position_track_enable_sub_;
    ros::Subscriber landing_condition_met_sub_;
    ros::Subscriber relanding_condition_met_sub_;
    ros::Subscriber obstacle_detection_sub_;

    double velocity_control_effort_x_;
    double velocity_control_effort_y_;
    double velocity_control_effort_yaw_;

    const double descending_speed_ = -0.5;
    const double ascending_speed_ = 0.5;

    bool obstacle_detected_ = false;
    bool position_track_enabled_ = false;
    bool landing_condition_met_ = false;
    bool relanding_condition_met_ = false;

    inline void velocityControlEffortXCallback(const std_msgs::Float64 &velocity_control_effort_x_msg)
    {
        velocity_control_effort_x_ = velocity_control_effort_x_msg.data;
    }

    inline void velocityControlEffortYCallback(const std_msgs::Float64 &velocity_control_effort_y_msg)
    {
        velocity_control_effort_y_ = velocity_control_effort_y_msg.data;
    }

    inline void velocityControlEffortYawCallback(const std_msgs::Float64 &velocity_control_effort_yaw_msg)
    {
        velocity_control_effort_yaw_ = velocity_control_effort_yaw_msg.data;
    }

    inline void positionTrackEnableCallback(const std_msgs::Bool &position_track_enable_msg)
    {
        position_track_enabled_ = position_track_enable_msg.data;
    }

    inline void landingConditionMetCallback(const std_msgs::Bool &landing_condition_met_msg)
    {
        landing_condition_met_ = landing_condition_met_msg.data;
    }

    inline void relandingConditionMetCallback(const std_msgs::Bool &relanding_condition_met_msg)
    {
        relanding_condition_met_ = relanding_condition_met_msg.data;
    }
};
}

#endif