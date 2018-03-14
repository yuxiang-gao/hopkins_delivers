#include "hd_control/hd_drone_interface.h"

namespace hd_interface
{
DroneInterface::DroneInterface(ros::NodeHandle *nh, ros::NodeHandle *nh_priv):
    nh_(*nh), nh_priv_(*nh_priv)
{
    ctrl_pub_ = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

    // Basic services
    sdk_ctrl_authority_service_ = nh_.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    drone_task_service_ = nh_.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service_ = nh_.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    set_local_pos_reference_ = nh_.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

    try
    {
        if (!obtainControl())
            throw 0;
        set_local_position();
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

bool DroneInterface::setLocalPosition()
{
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference_.call(localPosReferenceSetter);
}

bool DroneInterface::resleaseControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 0;
    sdk_ctrl_authority_service.call(authority);

    if (!authority.response.result)
    {
        ROS_ERROR("Release control failed!");
        return false;
    }

    return true;
}
} // namespace hd_interface