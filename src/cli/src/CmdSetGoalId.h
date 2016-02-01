#pragma once

#include "Cmd.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class CmdSetGoalId : public Cmd
{
public:
  CmdSetGoalId();

  bool match(std::string cmd);
  std::string run();
  std::string getPromptString();

private:
  std::string cmd_;
  const std::string match_ = "goal set";

  ros::NodeHandle node;

  int goal_id_;
  ros::Publisher goal_id_pub;
};
