#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pathfinder/Path.h>

#include "TargetDetermination.h"
#include "Planner.h"


int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Advertise paths topic
  ros::Publisher path_pub = node.advertise<pathfinder::Path>("/paths", 10);

  // Setup target determination
  TargetDetermination td;
  std::map<unsigned int, geometry_msgs::PoseStamped> goals;

  // Setup path planner
  Planner p;

  // Loop
  ros::Rate loop_rate(1.0);

  while (ros::ok())
  {
    // Get position of all aruco markers
    goals = td.getGoals();

    // Calculate path for each single goal
    for (const auto &goal : goals)
    {
      // Calc path
      nav_msgs::Path path = p.makePlan(goal.second);

      // debug
      p.debug_broadcast_tf(goal.first, path);


      // Publish path
      pathfinder::Path msg;

      msg.destination_id = goal.first;
      msg.path = path;

      path_pub.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
