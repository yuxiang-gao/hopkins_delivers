#include "hd_depth/obstacle_detection_class.h"
#include "hd_msgs/ObstacleDetection.h"
#define DEBUG 1
#if DEBUG
#include <chrono>
#endif
#define POT_FIELD 0

namespace hd_depth
{
ObstacleDetection::ObstacleDetection (ros::NodeHandle *nh, ros::NodeHandle *nh_priv, const std::string & name):
    nh_(*nh), 
    nh_priv_(*nh_priv), 
    it_(nh_), 
    openni_enc_(false), 
    name_(name), 
    depth_scale_(openni_enc_?1000.0:1.0),
    thresh_min_(0.5), 
    thresh_max_(2.0)
{
    ROS_DEBUG_ONCE_NAMED(name_, "Starting obstacle avoidance.");
    image_sub_ = it_.subscribe("depth_in", 1, &ObstacleDetection::imageCallback, this);
    // nh_priv_.param("openni_enc", openni_enc_, openni_enc_);
    // image_pub_ = it_.advertise("out", 1);
    obstacle_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/hd/perception/stereo_obstacle", 10);
    repulsive_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/hd/perception/potfield/stereo_obstacle", 10);

    if (openni_enc_) 
    {
        ROS_INFO_ONCE_NAMED(name_, "Using openni(16UC1) mode.");
        encoding_ = sensor_msgs::image_encodings::TYPE_16UC1;
    }
    else
    {
        ROS_INFO_ONCE_NAMED(name_, "Using zed(32FC1) mode.");
        encoding_ = sensor_msgs::image_encodings::TYPE_32FC1;
    }

} //constructor

void ObstacleDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
#if DEBUG
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, encoding_);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_NAMED(name_, "cv_bridge exception: %s", e.what());
        return;
    }
    // cv::Mat depth = cv_ptr->image;
    // depth.setTo(0.0, depth > 15.0);
    //typedef unsigned short PIXEL_TYPE;
    typedef float PIXEL_TYPE;
    cv::Mat_<PIXEL_TYPE> & depth = (cv::Mat_<PIXEL_TYPE> & )cv_ptr->image;
    //cv::Mat depth = cv_ptr->image;
    int offset_up = depth.rows / 10;
    int offset_down = offset_up;
    int offset_left = 0;
    int offset_right = 0;
    int cropped_height = depth.rows - offset_down - offset_up;
    int cropped_width = depth.cols - offset_left - offset_right;
    int num_grids = 6;
    const int num_rows = 3;

    Eigen::Vector3d repulsive_vector(0, 0, 0);
    //std::vector<int> grid_x = buildGrid1d(num_grids, cropped_width);
    std::vector<std::vector<int>> hist(num_rows, std::vector<int>(num_grids, 0));
    GridMap map = GridMap::Zero();
    //std::fill(hist.begin(), hist.end(), 0); 
    std::array<float, num_rows + 1> thresholds = {0.5, 2.0, 3.5, 5.0};
    int bin_width = cropped_width / num_grids;

#if POT_FIELD
    for ( int v=offset_up; v<depth.rows-offset_down; v++ )
    {
        for (int u=offset_left; u<depth.cols-offset_right; u++)
        {
            double d = (double)(depth.ptr<PIXEL_TYPE> ( v )[u]) / depth_scale_; // depth value in 16UC1
            if ( std::isnan(d) || d < thresholds[0] || d > thresholds[num_rows]) continue;
            Eigen::Vector3d point; 
            point[2] = d; 
            point[0] = (u - cx_) * point[2] / fx_;
            point[1] = (v - cy_) * point[2] / fy_;
            
            if (point[2] > 0.5 && point[2] <5.0 && fabs(point[0])< 2.0 && point[1] >0.0 && point[1] < 1.0)
            {
                double norm = point.norm();
                if (norm > potential_thresh_ || norm < 0.2) continue;
                repulsive_vector += point;
            }
        }  
    } 
    double repulsive_force = obstacle_gain_ * (1 / repulsive_vector.norm() - inv_potential_thresh_);
    geometry_msgs::PointStamped msg;
    constructPointMsg(repulsive_force * repulsive_vector.normalize(), msg);
    repulsive_pub_.publish(msg);
#else   
    for ( int v=offset_up; v<depth.rows-offset_down; v++ )
    {
        // six bins
        for (int i=0; i<num_grids; ++i)
        {
            for ( int u=offset_left+i*bin_width; u<(i==num_grids-1?depth.cols-offset_right:offset_left+(i+1)*bin_width); u++ )
            {
                //std::cout << "hxw: " <<depth.rows<<", " << depth.cols << ", " << bin_width << std::endl;
                float d = (float)(depth.ptr<PIXEL_TYPE> ( v )[u]) / depth_scale_; // depth value in 16UC1
                if ( std::isnan(d) || d < thresholds[0] || d > thresholds[num_rows]) 
                    continue;
                if ( d > thresholds[0] && d < thresholds[1]) 
                {
                    map(0, i)++;
                    map(1, i)++;
                    map(2, i)++;
                }
                else if ( d > thresholds[1] && d < thresholds[2])
                {
                    map(1, i)++;
                    map(2, i)++;
                }
                else if ( d > thresholds[2] && d < thresholds[3])
                {
                    map(2, i)++;
                }
            }   
        }
    }
    
    //hd_msgs::ObstacleDetection ob;
    //ob.header.stamp = ros::Time::now();
    ROS_DEBUG_STREAM_NAMED(name_, "hist: " << map );
    // for (int i=0; i<num_rows; i++)
    // {
    //     std::cout << "row[" << i <<"]: "; 
    //     for (const auto b: hist[i])
    //     {
    //         //ob.data.push_back(b);
    //         std::cout << b << ", ";
    //     }
    //     std::cout << ". " << std::endl;
    // }
    //obstacle_pub_.publish(ob);

#endif
#if DEBUG
    std::chrono::steady_clock::time_point t2 =std:: chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
    //OS_DEBUG_STREAM_NAMED(name_, "Time used: " << time_used.count() << " s.");
    std::cout<<"Time used: "<<time_used.count()<<" s."<<std::endl;
#endif

    //image_pub_.publish(cv_ptr->toImageMsg());
} // image_callback

void ObstacleDetection::constructPointMsg(const Eigen::Vector3d &point, geometry_msgs::PointStamped &msg)
{
    msg.header.stamp = ros::Time::now();
    msg.point.x = point[0];
    msg.point.y = point[1];
    msg.point.z = point[2];
}

void ObstacleDetection::constructMapMsg(const GridMap &map, nav_msgs::OccupancyGrid &msg)
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = 'odom';
    msg.info.width = 6;
    msg.info.height = 3;
    map = map / map.maxCoeff() * 100;
    int data[map.size()] = { }; 
    Eigen::Map<GridMap>(data, map.rows(), map.cols()) = map;
    msg.data = data;
}
} // namespace hd_depth
