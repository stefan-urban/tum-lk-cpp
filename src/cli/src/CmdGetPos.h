#pragma once

#include "Cmd.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class CmdGetPos : public Cmd
{
public:
  bool match(std::string cmd);
  std::string run();
  std::string getPromptString();

private:
  std::string cmd_;
  const std::string match_ = "pos get";

  ros::NodeHandle node;
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

  bool odomCheck;
  nav_msgs::Odometry odom_;
};
