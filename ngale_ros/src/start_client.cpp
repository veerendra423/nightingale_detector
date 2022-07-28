#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ngale_ros/start_service.h"
#include <cstdlib>
#include <iostream>
int main(int argc, char** argv)
{
  ros::init(argc,argv,"start_client");
  if(argc <= 1)
  {
    ROS_INFO("\nLess than number of expected arrguments");
  }
  ros::NodeHandle start_node;
  ros::ServiceClient client = start_node.serviceClient<ngale_ros::start_service>("start_ngale_pipeline");

  ngale_ros::start_service var;
  
  var.request.rgb_cam = atoi(argv[1]) ;
  var.request.rgb_det = atoi(argv[2]) ;
  var.request.thr_cam = atoi(argv[3]) ;
  var.request.thr_det = atoi(argv[4]) ;
  ROS_INFO("\n Sending the following values %d %d %d %d %s %s for service. ",(int)var.request.rgb_cam, (int)var.request.rgb_det, (int)var.request.thr_cam, (int)var.request.thr_det);

  if(client.call(var))
  {
    ROS_INFO("\nGot this response form service : %s",var.response.start_status.c_str());
    std::cout<<"\nRes"<<var.response.start_status;
  }
  else
  {
     ROS_INFO("\nFailed to call start_ngale_pipeline");
     return 1;
  }
  
  return 0;
}
