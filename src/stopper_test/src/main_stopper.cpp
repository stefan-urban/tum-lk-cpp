#include <ros/ros.h>

#include "Stopper.h"

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "turtlebot_stopper_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Start stopper
  Stopper stopper;
  stopper.startMoving();

  // Good bye
  ros::shutdown();

  return 0;
}
