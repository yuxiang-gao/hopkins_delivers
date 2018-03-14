#ifndef HD_INTERFACE_H
#define HD_INTERFACE_H

#include <ros/ros.h>
#include <ros/console.h>

#include <cmath>
#include <iostream>
#include <vector>
#include <iterator>
#include <string>

#include "dji_sdk/dji_sdk.h"
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

namespace hd_interface
{
class DroneInterface
{
  public:
    DroneInterface(ros::NodeHandle *nh, ros::NodeHandle *nh_priv);

  private:
    ~DroneInterface();
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_("~");

    ros::Publisher ctrl_pub_;

    ros::ServiceClient set_local_pos_reference_;
    ros::ServiceClient sdk_ctrl_authority_service_;
    ros::ServiceClient drone_task_service_;
    ros::ServiceClient query_version_service_;

    uint8_t flag_ = (DJISDK::VERTICAL_VELOCITY |
                     DJISDK::HORIZONTAL_VELOCITY |
                     DJISDK::YAW_RATE |
                     DJISDK::HORIZONTAL_BODY |
                     //DJISDK::VERTICAL_BODY   |
                     DJISDK::STABLE_ENABLE);

    //std::string topic_from_controller_;

    void sendControlSignal(double x, double y, double z, double yaw);
    bool obtainControl();
    bool releaseControl();

    // inline void velocityControlEffortXCallback(const std_msgs::Float64 &velocity_control_effort_x_msg)
    // {
    //     velocity_control_effort_x_ = velocity_control_effort_x_msg.data;
    // }

    // inline void velocityControlEffortYCallback(const std_msgs::Float64 &velocity_control_effort_y_msg)
    // {
    //     velocity_control_effort_y_ = velocity_control_effort_y_msg.data;
    // }

    // inline void velocityControlEffortYawCallback(const std_msgs::Float64 &velocity_control_effort_yaw_msg)
    // {
    //     velocity_control_effort_yaw_ = velocity_control_effort_yaw_msg.data;
    // }

    // inline void positionTrackEnableCallback(const std_msgs::Bool &position_track_enable_msg)
    // {
    //     position_track_enabled_ = position_track_enable_msg.data;
    // }

    // inline void landingConditionMetCallback(const std_msgs::Bool &landing_condition_met_msg)
    // {
    //     landing_condition_met_ = landing_condition_met_msg.data;
    // }

    // inline void relandingConditionMetCallback(const std_msgs::Bool &relanding_condition_met_msg)
    // {
    //     relanding_condition_met_ = relanding_condition_met_msg.data;
    // }

    //inline void obstacleCallback(const )
}; // class DroneInterface

typedef boost::shared_ptr<DroneInterface> DroneInterfacePtr;
} // namespace hd_interface

#endif
