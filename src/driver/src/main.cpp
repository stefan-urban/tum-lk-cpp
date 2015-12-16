#include <ros/ros.h>

#include "Rotator.h"

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "driver_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  ros::Rate loop_rate(10.0);

  Rotator rot;

  // Loop
  while (ros::ok())
  {
    // Rotate until ArUco code is detected
//    rot.rotateUnilCondition(void *function_handle);
    rot.rotateAngle(10);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye
  ros::shutdown();

  return 0;
}
