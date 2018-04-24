#include "hd_control/hd_control.h"
#define TEST 1
#define MISSION_TEST 0
namespace hd_control
{
DroneControl::DroneControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv) : nh_(*nh), nh_priv_(*nh_priv), drone_state_(std::make_pair(StateConst::DroneState::STATE_ON_GROUND, StateConst::PackageState::PACKAGE_OFF))
{
    // initiate drone interface
    drone_interface_ptr_.reset(new hd_interface::DroneInterface(nh, nh_priv));

    attitude_sub_ = nh_.subscribe("dji_sdk/attitude", 10, &DroneControl::attitudeCallback, this);
    gps_sub_ = nh_.subscribe("dji_sdk/gps_position", 10, &DroneControl::gpsCallback, this);
    flight_status_sub_ = nh_.subscribe("dji_sdk/flight_status", 10, &DroneControl::flightStatusCallback, this);

    velocity_control_x_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_x", 10, &DroneControl::velocityControlEffortXCallback, this);
    velocity_control_y_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_y", 10, &DroneControl::velocityControlEffortYCallback, this);
    velocity_control_yaw_sub_ = nh_.subscribe("/hd/position_track/velocity_control_effort_yaw", 10, &DroneControl::velocityControlEffortYawCallback, this);
    
    position_track_enable_sub_ = nh_.subscribe("/hd/position_track/position_track_enable", 1, &DroneControl::positionTrackEnableCallback, this);
    landing_condition_met_sub_ = nh_.subscribe("/hd/position_track/landing_condition_met", 1, &DroneControl::landingConditionMetCallback, this);
    relanding_condition_met_sub_ = nh_.subscribe("/hd/position_track/relanding_condition_met", 1, &DroneControl::relandingConditionMetCallback, this);
    
    obstacle_detection_sub_ = nh_.subscribe("/hd/perception/stereo_obstacles", 10, &DroneControl::obstacleCallback, this);

    repulsive_force_sub_ = nh_.subscribe("/hd/perception/repulsive_force", 10, &DroneControl::repulsiveForceCallback, this);

    // grab package & take off
    try
    {
        if (!drone_interface_ptr_->grabPackage())
        {   
            //throw 0;
            ROS_INFO("Grab");
        }
        else
        {
            ROS_INFO("Grab package succeed!");
            drone_state_.second = StateConst::PackageState::PACKAGE_ON;
        }
        ros::Duration(5.0).sleep();

        if (!monitoredTakeoff())
            throw 1;
        else
        {
            ROS_INFO("Takeoff succeed!");
            drone_state_.first = StateConst::DroneState::STATE_IN_AIR;
        }
           
        
    }
    catch (int e)
    {
        if (e==0)
            ROS_ERROR("Grab package FAILED!");
        else if (e==1)
            ROS_ERROR("Takeoff FAILED!");
    }

    // set goal


    //
#if TEST
    while (ros::ok())
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
                drone_interface_ptr_->sendControlSignal(0,0 , 0.5, velocity_control_effort_yaw_);
                //drone_interface_ptr_->sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, descending_speed_, velocity_control_effort_yaw_);
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
        else
            continue;
    }
#endif

#if MISSION_TEST
    while (ros::ok())
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
        else
            continue;
    }
#endif
}

DroneControl::~DroneControl()
{
    drone_interface_ptr_.reset();
}

void DroneControl::repulsiveForceCallback(const geometry_msgs::PointStamped::ConstPtr &rep)
{
    ROS_INFO("haha");
}

void DroneControl::obstacleCallback(const hd_msgs::ObstacleDetection::ConstPtr &ob)
{
    double ob_mid_bins = 0;
    int num_bins = ob->num_bins;
    if (num_bins % 2 == 0)
    {
        ob_mid_bins = ob->data[num_bins / 2] + ob->data[num_bins / 2 + 1];
        ob_mid_bins /= 2;
    }
    else
    {
        ob_mid_bins = ob->data[num_bins / 2 + 1];
    }

    obstacle_running_average_ = (1 - obstacle_alpha_) * obstacle_running_average_ + obstacle_alpha_ * ob_mid_bins;

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
        for (int i = 0; i < num_bins; i++)
        {
            if ((i + 1) <= num_bins / 2)
                ob_left += ob->data[i];
            else
                ob_right += ob->data[i];
        }
        if (ob_left < ob_right)
        {
            ROS_DEBUG("Obstcle Detected. Moving left.");
            drone_interface_ptr_->sendControlSignal(0.0, -0.2, 0.0, 0.0);
        }
        else 
        {
            ROS_DEBUG("Obstcle Detected. Moving right.");
            drone_interface_ptr_->sendControlSignal(0.0, 0.2, 0.0, 0.0);
        }
    }
    else
    {
        obstacle_detected_ = false;
    }
}

bool DroneControl::monitoredTakeoff()
{
    ros::Time start_time = ros::Time::now();

    float home_altitude = current_gps_.altitude;
    if (!drone_interface_ptr_->takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
    {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1: If M100 is not in the air after 10 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(10))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
    double  alt = current_gps_.altitude - home_altitude;
     ROS_DEBUG_STREAM("alt :" << alt);
    if (flight_status_ != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR || current_gps_.altitude - home_altitude < 1.0)
    {
        ROS_ERROR("Takeoff failed.");
        return false;
    }
    else
    {
        start_time = ros::Time::now();
        ROS_INFO("Successful takeoff!");
        ros::spinOnce();
    }

    return true;
}
} // namespace hd_control

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hd_drone_control");
    ROS_INFO("Starting Drone Controller.");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    hd_control::DroneControl drone_control(&nh, &nh_priv);
}
