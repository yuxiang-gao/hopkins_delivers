#ifndef HD_DEPTH_OBSTACLE_DETECTION_NODELET_H
#define HD_DEPTH_OBSTACLE_DETECTION_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "hd_depth/obstacle_detection_class.h"

namespace hd_depth
{
class ObstacleDetectionNodelet: public nodelet::Nodelet
{

public:
    ObstacleDetectionNodelet(){}
private:
    ~ObstacleDetectionNodelet(){}
    virtual void onInit();
    boost::shared_ptr<ObstacleDetection> inst_;

}; // class ObstacleDetectionNodelet
} // namespace hd_depth
#endif