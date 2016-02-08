#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

#include <pathfinder/Path.h>
#include <goalfinder/Goals.h>

#include "Planner.h"


std::vector<goalfinder::Goal> goals(8);

void goalsCallback(const goalfinder::GoalsConstPtr& goals_msg)
{
  if (goals_msg->list.size() == 0)
  {
    return;
  }

  std::vector<goalfinder::Goal> list = goals_msg->list;

  for (const auto &goal : list)
  {
    goals[goal.destination_id] = goal;
  }
}

geometry_msgs::PoseStamped current_pose;

void positionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  ROS_INFO_STREAM("Did receive current pose");

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

  // Periodically update own and goals' positions
  ros::Subscriber pose_subscriber = node.subscribe("/amcl_pose", 1, &positionCallback);
  ros::Subscriber goal_subscriber = node.subscribe("/goals", 1, &goalsCallback);

  // Wait for first position update
  while (ros::ok() && current_pose.header.stamp.sec == 0)
  {
    ROS_INFO("Wait for AMCL");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  // Advertise paths topic
  ros::Publisher path_pub = node.advertise<pathfinder::Path>("/paths", 10);

  // Setup planner
  Planner p;


  ROS_INFO("Startup finished!");

  // Loop
  ros::Rate loop_rate(0.5);

  while (ros::ok())
  {
    // Paths
    for (const auto& goal : goals)
    {
      // Calc path
      nav_msgs::Path path = p.makePlan(current_pose, goal.pose);

      // Not working, why ever
      if (path.poses.size() == 0)
      {
        continue;
      }

      // Publish path
      pathfinder::Path msg;

      msg.destination_id = goal.destination_id;
      msg.path = path;

      path_pub.publish(msg);

      ROS_INFO_STREAM("Published plan for goal #" << goal.destination_id);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
