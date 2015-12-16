#include <ros/ros.h>

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "aruco_code_detection_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  ros::Rate loop_rate(50.0);

  // Loop
  while (ros::ok())
  {


    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye
  ros::shutdown();

  return 0;
}
