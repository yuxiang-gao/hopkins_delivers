#include "ros/ros.h"
#include <hd_msgs/EMPick.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EM_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hd_msgs::EMPick>("EM");
  hd_msgs::EMPick srv;
  srv.request.command = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("SUCCESSFULLY PICK UP");
  }
  else
  {
    ROS_INFO("Drop Package ");
    return 1;
  }

  return 0;
}
