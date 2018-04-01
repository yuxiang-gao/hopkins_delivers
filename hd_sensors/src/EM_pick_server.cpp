#include  <ros/ros.h>
#include <std_msgs/Bool.h>
#include <hd_msgs/EMPick.h>

bool EMService_request( hd_msgs::EMPick::Request &req, hd_msgs::EMPick::Response &res)
 {
	 if( req.command == true )
	{
            res.result = true;
            ROS_INFO("Em are now working to pick up packages");
            return true;
   	}
  	else
	{
           res.result = false;
            ROS_INFO("Shut down EMs");
           return false;
  	}
}
	 
	 // Testing
	 
int main( int argc, char** argv)
{
	
  ros::init(argc, argv, "EM_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("EM", EMService_request);
  ROS_INFO("Ready to pick up.");
  ros::spin();

   return 0;
}
