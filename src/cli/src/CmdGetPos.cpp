
#include "CmdGetPos.h"

#include <chrono>
#include <sstream>
#include <thread>

bool CmdGetPos::match(std::string cmd)
{
  if (match_.compare(cmd) == 0)
  {
    return true;
  }

  return false;
}

std::string CmdGetPos::run()
{
  odomCheck = false;

  // First, subscribe to odometry
  ros::Subscriber sub = node.subscribe("/odom", 1, &CmdGetPos::odomCallback, this);

  // Start timeout clock
  auto t_start = std::chrono::high_resolution_clock::now();

  // Wait 3 seconds
  do
  {
    // Success
    if (odomCheck == true)
    {
      // Stop listening to topic
      sub.shutdown();

      // Format info
      std::ostringstream oss;

      oss << "position:\n";
      oss << " - x: " << odom_.pose.pose.position.x << "\n";
      oss << " - y: " << odom_.pose.pose.position.x << "\n";
      oss << " - z: " << odom_.pose.pose.position.x << "";

      return oss.str();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } while (std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count() < 3000);

  return std::string("error, no odometry received");
}

std::string CmdGetPos::getPromptString()
{
  return "pos get\tReturns the current turtlebot position";
}

void CmdGetPos::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  // Copy
  odom_ = *odom;

  // Set flag
  odomCheck = true;
}
