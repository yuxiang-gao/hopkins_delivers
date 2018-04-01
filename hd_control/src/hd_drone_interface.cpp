#include "hd_control/hd_drone_interface.h"

namespace hd_interface
{
DroneInterface::DroneInterface(ros::NodeHandle *nh, ros::NodeHandle *nh_priv) : nh_(*nh), nh_priv_(*nh_priv)
{
    ctrl_pub_ = nh_.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

    // Basic services
    sdk_ctrl_authority_service_ = nh_.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    drone_task_service_ = nh_.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service_ = nh_.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    set_local_pos_reference_ = nh_.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
    
    grab_package_service_ = nh_.serviceClient<hd_msgs::EMPick>("/hd/package/em_control");

    try
    {
        if (!obtainControl())
            throw 0;
        setLocalPosition();
        ROS_INFO("Obtain control succeed. Setting local pos!");
    }
    catch (int e)
    {
        ROS_ERROR("Obtain control FAILED!");
    }
}

DroneInterface::~DroneInterface()
{
    if (releaseControl())
        ROS_INFO("Release control succeed.");
}

void DroneInterface::sendControlSignal(double x, double y, double z, double yaw_rate)
{
    sensor_msgs::Joy control_pos_yaw_rate;
    control_pos_yaw_rate.axes.push_back(x);
    control_pos_yaw_rate.axes.push_back(y);
    control_pos_yaw_rate.axes.push_back(z);
    control_pos_yaw_rate.axes.push_back(yaw_rate);
    control_pos_yaw_rate.axes.push_back(flag_);

    ctrl_pub_.publish(control_pos_yaw_rate);
}

bool DroneInterface::grabPackage()
{
    hd_msgs::EMPick em_service;
    em_service.request.command = true;
    grab_package_service_.call(em_service);
    if (!em_service.response.result)
    {
        ROS_ERROR("Grab package failed!");
        return false;
    }
    return true;
}

bool DroneInterface::releasePackage()
{
    hd_msgs::EMPick em_service;
    em_service.request.command = false;
    grab_package_service_.call(em_service);
    if (!em_service.response.result)
    {
        ROS_ERROR("Release package failed!");
        return false;
    }
    return true;
}

bool DroneInterface::obtainControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    sdk_ctrl_authority_service_.call(authority);

    if (!authority.response.result)
    {
        ROS_ERROR("obtain control failed!");
        return false;
    }

    return true;
}

bool DroneInterface::releaseControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 0;
    sdk_ctrl_authority_service_.call(authority);

    if (!authority.response.result)
    {
        ROS_ERROR("Release control failed!");
        return false;
    }

    return true;
}

bool DroneInterface::setLocalPosition()
{
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference_.call(localPosReferenceSetter);
}

bool DroneInterface::takeoffLand(int task)
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    drone_task_service_.call(droneTaskControl);

    if (!droneTaskControl.response.result)
    {
        ROS_ERROR("takeoff_land fail");
        return false;
    }

    return true;
}

} // namespace hd_interface
