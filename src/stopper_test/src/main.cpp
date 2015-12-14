#include <ros/ros.h>

#include "Stopper.h"
#include "Rotator.h"

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "turtlebot_aimless_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  ros::Rate loop_rate(10);

  Stopper stopper;
  Rotator rotator;

  while (ros::ok())
  {
    ROS_INFO_STREAM("Start moving");
    stopper.startMoving();

    ROS_INFO_STREAM("Start rotation by 20 degrees");
    rotator.startRotation(20.0 / 180.0 * M_PI);

    ros::spinOnce();
    loop_rate.sleep();
}

  // Good bye
  ros::shutdown();

  return 0;
}
