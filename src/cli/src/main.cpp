#include <ros/ros.h>
#include <thread>

#include "CommandLine.h"
#include "CmdGetPos.h"

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "cli");
  ros::NodeHandle node;

  // Start ROS
  ros::start();

  // Register all commands
  CommandLine cmd_line;

  cmd_line.registerCmd((Cmd*) new CmdGetPos());

  // Start thread for command input
  std::thread cmd_input(&CommandLine::start, &cmd_line);

  // Just run
  ros::spin();

  // Wait for threads to finish
  cmd_line.stop();
  cmd_input.join();

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
