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
#include <std_msgs/Bool.h>
#include <hd_msgs/EMService.h>

#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

namespace hd_control
{
class DroneInterface
{
  public:
    DroneInterface(ros::NodeHandle *nh, ros::NodeHandle *nh_priv);
    ~DroneInterface();
    void sendControlSignal(double x, double y, double z, double yaw, bool use_rate = true);
    void sendENUControlSignal(double x, double y, double z, double yaw, bool use_rate = true);
    bool grabPackage();
    bool releasePackage();
    bool takeoffLand(int task);

  private:
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    ros::Publisher ctrl_pub_;

    ros::ServiceClient set_local_pos_reference_;
    ros::ServiceClient sdk_ctrl_authority_service_;
    ros::ServiceClient drone_task_service_;
    ros::ServiceClient query_version_service_;

    ros::ServiceClient grab_package_service_;

    uint8_t flag_fru_rate_ = (DJISDK::HORIZONTAL_VELOCITY |
                     DJISDK::VERTICAL_VELOCITY |
                     DJISDK::YAW_RATE |
                     DJISDK::HORIZONTAL_BODY |
                     DJISDK::STABLE_ENABLE);
    uint8_t flag_enu_rate_ = (DJISDK::HORIZONTAL_VELOCITY |
                     DJISDK::VERTICAL_VELOCITY |
                     DJISDK::YAW_RATE |
                     DJISDK::HORIZONTAL_GROUND |
                     DJISDK::STABLE_ENABLE);
    uint8_t flag_fru_angle_ = (DJISDK::HORIZONTAL_VELOCITY |
                     DJISDK::VERTICAL_VELOCITY |
                     DJISDK::YAW_ANGLE  |
                     DJISDK::HORIZONTAL_BODY |
                     DJISDK::STABLE_ENABLE);
    uint8_t flag_enu_angle_ = (DJISDK::HORIZONTAL_VELOCITY |
                     DJISDK::VERTICAL_VELOCITY |
                     DJISDK::YAW_ANGLE |
                     DJISDK::HORIZONTAL_GROUND |
                     DJISDK::STABLE_ENABLE);

    bool obtainControl();
    bool releaseControl();
    bool setLocalPosition();
    

}; // class DroneInterface

typedef boost::shared_ptr<DroneInterface> DroneInterfacePtr;
} // namespace hd_interface

#endif
