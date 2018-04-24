#include "hd_control/hd_control.h"
#define TEST 1
#define MISSION_TEST 1
namespace hd_control
{
DroneControl::DroneControl(ros::NodeHandle *nh, ros::NodeHandle *nh_priv) : nh_(*nh), nh_priv_(*nh_priv), drone_state_(DroneStates::STATE_ON_GROUND, false)
{
    // initiate drone interface
    drone_interface_ptr_.reset(new DroneInterface(nh, nh_priv));
    mission_ptr_.reset(new Mission(drone_interface_ptr_));

    attitude_sub_ = nh_.subscribe("dji_sdk/attitude", 10, &DroneControl::attitudeCallback, this);
    gps_sub_ = nh_.subscribe("dji_sdk/gps_position", 10, &DroneControl::gpsCallback, this);
    flight_status_sub_ = nh_.subscribe("dji_sdk/flight_status", 10, &DroneControl::flightStatusCallback, this);
    local_position_sub_ = nh.subscribe("dji_sdk/local_position", 10, &DroneControl::localPositionCallback, this);

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
            drone_state_.package_state = true;
        }
        ros::Duration(5.0).sleep();

        if (!monitoredTakeoff())
            throw 1;
        else
        {
            ROS_INFO("Takeoff succeed!");
            drone_state_.drone_state = DroneState::STATE_IN_AIR;
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
    ros::spinOnce();
    mission_ptr_.reset(new Mission(drone_interface_ptr_, &current_gps_, &current_local_pos_));
    FlightTarget flight_target;
    flight_target.x = 5;
    std::vector<FlightTarget> flight_plan;
    flight_plan.push_back(flight_target);
    mission_ptr_->setPlan(flight_plan);

    ros::Rate loop_rate(50);
    


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
                // drone_interface_ptr_->sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, descending_speed_, velocity_control_effort_yaw_);
                monitoredLanding();
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
        Eigen::Vector3d v(0, 0, 0);
        if (obstacle_detected_)
        {
            double power = obstacle_avoid_speed_ / (obstacle_dist_ + 1);
            if (obstacle_dir_ == 0)
                v(1) += power;
            else
                v(1) -= power;

        }
        else if (position_track_enabled_)
        {
            if (landing_condition_met_)
            {
                monitoredLanding();
                // drone_interface_ptr_->sendControlSignal(velocity_control_effort_x_, velocity_control_effort_y_, descending_speed_, velocity_control_effort_yaw_);
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
        loop_rate.sleep();
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

void DroneControl::obstacleCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    int width = map->info.width;
    int height = map->info.height;
    int center = map->info.origin.position.x;
    hd_depth::MapTypeConst m(map->data);
    if (m.block<3, 3>(0, 0).sum() < m.block<3, 3>(0, 3).sum())
        obstacle_dir_ = 1;
    else
        obstacle_dir_ = 0;
    // **00**
    // *0000*
    // 000000
    if ( m.row(0).maxCoeff() < obstacle_threshold_ )
    {
        if (m.block<1, 4>(1, 1).maxCoeff() < obstacle_threshold_)
        {
            if (m.block<1, 2>(2, 2).maxCoeff() < obstacle_threshold_)
            {
                obstacle_detected_ = false;
                obstacle_dist_ = 0;
            }
            else
            {
                obstacle_detected_ = true;
                obstacle_dist_ = 5;
            }
        }
        else
        {
            obstacle_detected_ = true;
            obstacle_dist_ = 3;
        }
    }
    else
    {
        obstacle_detected_ = true;
        obstacle_dist_ = 1;
    }
}

void DroneControl::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    current_gps_ = *msg;

    // // Down sampled to 50Hz loop
    // if(elapsed_time > ros::Duration(0.02))
    // {
    //     start_time = ros::Time::now();
    //     switch(mission_ptr_->state)
    //     {
    //     case 0:
    //         break;

    //     case 1:
    //         if(!mission_ptr_->finished)
    //         {
    //             mission_ptr_->step();
    //         }
    //         else
    //         {
    //             mission_ptr_->reset();
    //             mission_ptr_->start_gps_location = current_gps;
    //             mission_ptr_->setTarget(20, 0, 0, 0);
    //             mission_ptr_->state = 2;
    //             ROS_INFO("##### Start route %d ....", mission_ptr_->state);
    //         }
    //         break;
    //     }
    // }
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

    if (flight_status_ != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
        current_gps_.altitude - home_altitude < 1.0)
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

bool DroneControl::monitoredLanding()
{
    ros::Time start_time = ros::Time::now();

    float home_altitude = current_gps_.altitude;
    if (!drone_interface_ptr_->takeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LANDING))
    {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1: If M100 is not in the air after 10 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(10) || flight_status_ == DJISDK::M100FlightStatus::M100_STATUS_LANDING)
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (flight_status_ != DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND ||
        current_gps_.altitude - home_altitude > 1.0)
    {
        ROS_ERROR("Landing failed.");
        return false;
    }
    else
    {
        start_time = ros::Time::now();
        ROS_INFO("Successful Landed!");
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
