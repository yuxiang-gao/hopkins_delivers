#include "hd_control/hd_drone_interface.h"

namespace hd_control
{
DroneInterface::DroneInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
    //nh_priv_.param<std::string>("topic_from_controller", topic_from_controller_, "/hd/position_track/velocity_control_effort_x");

    // velocity_control_x_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_x", 10, &velocityControlEffortXCallback);
    // velocity_control_y_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_y", 10, &velocityControlEffortYCallback);
    // velocity_control_yaw_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_yaw", 10, &velocityControlEffortYawCallback);
    // position_track_enable_sub_ = nh_.subscribe("/hd/position_track/position_track_enable", 1, &positionTrackEnableCallback);
    // landing_condition_met_sub_ = nh_.subscribe("/hd/position_track/landing_condition_met", 1, &landingConditionMetCallback);
    // relanding_condition_met_sub_ = nh_.subscribe("/hd/position_track/relanding_condition_met", 1, &relandingConditionMetCallback);
    // obstacle_detection_sub_ = nh_.subscribe("/hd/obstacle_avoidance/obstacles", 10, &obstcleCallback);

    ctrl_pub_ = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

    // Basic services
    sdk_ctrl_authority_service_ = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    drone_task_service_ = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service_ = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    set_local_pos_reference_ = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

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
}