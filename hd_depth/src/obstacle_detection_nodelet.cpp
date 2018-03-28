#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "hd_depth/obstacle_detection_nodelet.h"

PLUGINLIB_EXPORT_CLASS(hd_depth::ObstacleDetectionNodelet, nodelet::Nodelet)
namespace hd_depth
{
void ObstacleDetectionNodelet::onInit()
{
    NODELET_DEBUG("Initializing nodelet");
    inst_.reset(new ObstacleDetection(&getNodeHandle(), &getPrivateNodeHandle(), getName()));
} // class 
} // namespace hd_depth
