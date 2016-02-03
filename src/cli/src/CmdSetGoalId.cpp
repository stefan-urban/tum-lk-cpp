
#include "CmdSetGoalId.h"

#include <chrono>
#include <sstream>
#include <thread>
#include <std_msgs/Int32.h>


CmdSetGoalId::CmdSetGoalId()
{
  goal_id_pub = node.advertise<std_msgs::Int32>("/cli/goals/set_id", 10);
}

bool CmdSetGoalId::match(std::string cmd)
{
  if (match_.compare(cmd.substr(0, match_.length())) == 0)
  {
    // Try to parse goal id
    try
    {
      goal_id_ = std::stoi(cmd.substr(match_.length()));
    }
    catch (std::invalid_argument e)
    {
      return false;
    }

    return true;
  }

  return false;
}

std::string CmdSetGoalId::run()
{
  std_msgs::Int32 msg;
  msg.data = goal_id_;

  goal_id_pub.publish(msg);

  std::ostringstream oss;
  oss << "Did set goal id " << goal_id_;
  return oss.str();
}

std::string CmdSetGoalId::getPromptString()
{
  return "goal set\tSets current destination id";
}
