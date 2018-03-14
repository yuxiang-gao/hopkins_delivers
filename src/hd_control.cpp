#include "hd_control/hd_control.h"

namespace hd_control
{
DroneControl::DroneControl(ros::NodeHandle &nh, ros::NodeHandle &nh_priv):
    nh_(*nh), nh_priv_(*nh_priv)
{
    drone_interface_ptr_.reset(new DroneInterface(nh, nh_priv));

    velocity_control_x_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_x", 10, &velocityControlEffortXCallback);
    velocity_control_y_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_y", 10, &velocityControlEffortYCallback);
    velocity_control_yaw_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_yaw", 10, &velocityControlEffortYawCallback);
    position_track_enable_sub_ = nh_.subscribe("/hd/position_track/position_track_enable", 1, &positionTrackEnableCallback);
    landing_condition_met_sub_ = nh_.subscribe("/hd/position_track/landing_condition_met", 1, &landingConditionMetCallback);
    relanding_condition_met_sub_ = nh_.subscribe("/hd/position_track/relanding_condition_met", 1, &relandingConditionMetCallback);
    obstacle_detection_sub_ = nh_.subscribe("/hd/obstacle_avoidance/obstacles", 10, &obstcleCallback);

    while(ros::ok())
    {
        ros::spinOnce();
        if (obstacle_detected_)
        {
            continue;
        }
        else if (position_track_enabled_)
        {
            if (landing_condition_met_)
            {
                drone_interface_ptr_->sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, descending_speed_, velocity_control_effort_yaw_);
            }
            else if (relanding_condition_met_)
            {
                drone_interface_ptr_->sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, ascending_speed_, velocity_control_effort_yaw_);
            }
            else
            {
                drone_interface_ptr_->sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, 0.0, velocity_control_effort_yaw_);
            }
        }
        else continue;
    }
}

DroneControl::~DroneControl()
{
    drone_interface_ptr_.reset();
}

void DroneControl::obstacleCallback(const hd_msgs::ObstacleDetection::ConstPtr &ob)
{
    double ob_mid_bins = 0;
    int num_bins = ob->num_bins;
    if (num_bins%2==0)
    {
        ob_mid_bins = ob->data[num_bins / 2] + ob->data[num_bins / 2 + 1];
        ob_mid_bins /= 2;
    }
    else
    {
        ob_mid_bins = ob->data[num_bins / 2 + 1];
    }

    obstacle_running_average_ = (1 - obstacle_alpha) * obstacle_running_average_ + obstacle_alpha_ * ob_mid_bins;

    if (obstacle_running_average_ > obstacle_threshold_)
    {
        obstacle_detected_ = true;
        // std::vector<int> ob_cnt(num_bins-2);
        // for (int i=0; i<num_bins-2; i++)
        // {
        //     ob_cnt[i] = ob->data[i] + ob->data[i+1] + ob->data[i+2];
        // }
        // int min_dir = std::distance(ob_cnt.begin(), std::min_element(ob_cnt.begin(), ob_cnt.end()));
        // if (min_dir <= (double)num_bins/2)
        // {
            
        // }
        int ob_left = 0;
        int ob_right = 0;
        for (int i=0; i<num_bins; i++)
        {
            if ((i+1)<=num_bins/2)
                ob_left += ob->data[i];
            else
                ob_right += ob->data[i];
        }
        if (ob_left < ob_right)
            drone_interface_ptr_->sendControlSignal(0.0, -0.2, 0.0, 0.0);
        else
            drone_interface_ptr_->sendControlSignal(0.0, 0.2, 0.0, 0.0);

    }
    else
    {
        obstacle_detected_ = false;
    }
    
}
} // namespace hd_controldrone_interface_ptr_->sendControlSignal(0.0, -0.2, 0.0, 0.0);