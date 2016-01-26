#include <ros/ros.h>
#include <thread>

#include "CommandLine.h"

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "cli");
  ros::NodeHandle node;

  // Start ROS
  ros::start();


  // Start thread for command input
  std::thread cmd_input(&CommandLine::start, CommandLine());

  // Just run
  ros::spin();

  // Good bye turtlebot
  ros::shutdown();

  // Wait for threads to finish
  cmd_input.join();

  return 0;
}
