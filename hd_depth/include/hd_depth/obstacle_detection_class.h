#ifndef HD_DEPTH_OBSTACLE_DETECTION_CLASS_H
#define HD_DEPTH_OBSTACLE_DETECTION_CLASS_H

#include <vector>
#include <cmath>
#include <ros/ros.h>
//#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 


namespace hd_depth
{
class ObstacleDetection
{
public: 
    ObstacleDetection(ros::NodeHandle *nh, ros::NodeHandle *nh_priv, const std::string & name);
    virtual ~ObstacleDetection(){};   
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    // image_transport::Publisher image_pub_; 
    ros::Publisher obstacle_pub_;
    ros::Publisher repulsive_pub_;
    std::string encoding_;
    std::string name_;
    bool openni_enc_;
    float depth_scale_;
    double thresh_min_, thresh_max_;

    // intrinsic
    const double cx_ = 325.5;
    const double cy_ = 253.5;
    const double fx_ = 518.0;
    const double fy_ = 519.0;

    // obstacle potential thresh
    
    const double potential_thresh_ = 2.0;
    const double inv_potential_thresh_ = 1.0 / potential_thresh_;
    double obstacle_gain_ = 0.5;
    //const float DEPTH_SCALE = 1000.0;

    // inline std::vector<int> buildGrid1d(const int & span, const int & num_grids)
    // {
    //     std::vector<int> grid;
    //     for (int i=0; i<num_grids+1; i++)
    //     {
    //         grid.push_back(span * i / num_grids);
    //     }
    //     return grid;
    // }
    void constructPointMsg(const Eigen::Vector3d &point, geometry_msgs::PointStamped &msg);
}; // class ObstacleDetection
} // namespace hd_depth

#endif
