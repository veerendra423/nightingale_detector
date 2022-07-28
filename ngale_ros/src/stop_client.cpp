#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ngale_ros/stop_service.h"
#include <cstdlib>
#include <iostream>
int main(int argc, char** argv)
{
  ros::init(argc,argv,"stop_client");
  if(argc <= 1)
  {
    ROS_INFO("\nLess than number of expected arrguments");
  }
  ros::NodeHandle stop_node;
  ros::ServiceClient client = stop_node.serviceClient<ngale_ros::stop_service>("stop_ngale_pipeline");

  ngale_ros::stop_service var;
  
  var.request.terminate = 1 ;
  ROS_INFO("\n Sending the following values terminate %d for stop service. ",(int)var.request.terminate);

  if(client.call(var))
  {
    ROS_INFO("\nGot this response form service : %s",var.response.stop_status.c_str());
  }
  else
  {
     ROS_INFO("\nFailed to call stop_ngale_pipeline");
     return 1;
  }
  
  return 0;
}
