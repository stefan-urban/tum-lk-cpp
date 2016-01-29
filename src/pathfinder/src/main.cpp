#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pathfinder/Path.h>
#include <pathfinder/Goals.h>

#include "TargetDetermination.h"
#include "Planner.h"


geometry_msgs::PoseStamped current_pose;

void positionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  current_pose.header = pose->header;
  current_pose.pose = pose->pose.pose;
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  // Start ROS
  ros::start();

  // Periodically update own position
  ros::Subscriber pose_subscriber = node.subscribe("/amcl_pose", 1, &positionCallback);

  // Wait for first position update
  while (ros::ok() && current_pose.header.stamp.sec == 0)
  {
    ROS_INFO("Wait for AMCL");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  // Advertise paths topic
  ros::Publisher path_pub = node.advertise<pathfinder::Path>("/paths", 10);
  ros::Publisher goals_pub = node.advertise<pathfinder::Goals>("/goals", 10);

  // Setup target determination
  TargetDetermination td;
  std::map<unsigned int, geometry_msgs::PoseStamped> goals;

  // Setup path planner
  Planner p;


  ROS_INFO("Startup finished!");

  // Loop
  ros::Rate loop_rate(1.0);

  while (ros::ok())
  {
    // Goals
    auto goals = td.getGoals();

    pathfinder::Goals goals_msg;

    for (const auto& goal : goals)
    {
      pathfinder::Goal goal_msg;

      goal_msg.destination_id = goal.first;
      goal_msg.pose = goal.second;

      goals_msg.list.push_back(goal_msg);
    }

    goals_pub.publish(goals_msg);

    td.broadcastTf();

    // Paths
    for (const auto& goal : goals)
    {
      // Calc path
      nav_msgs::Path path = p.makePlan(current_pose, goal.second);

      // Debug
      //p.debug_broadcast_tf(goal.first, path);

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
