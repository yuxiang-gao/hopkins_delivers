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
    void sendControlSignal(double x, double y, double z, double yaw);
    bool monitoredTakeoff();

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

    bool obtainControl();
    bool releaseControl();
    bool setLocalPosition();
    bool takeoffLand(int task);

}; // class DroneInterface

typedef boost::shared_ptr<DroneInterface> DroneInterfacePtr;
} // namespace hd_interface

#endif
