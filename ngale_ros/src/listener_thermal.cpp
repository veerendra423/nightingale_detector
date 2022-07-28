
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  printf("\nI heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_thermal");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("thermal_detect", 1000, chatterCallback);
  printf("\n\n STRATING SPIN\n");
  ros::spin();

  return 0;
}
