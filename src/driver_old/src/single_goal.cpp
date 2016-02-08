#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <goalfinder/Goals.h>

std::vector<goalfinder::Goal> goals;

void goalsCallback(const goalfinder::GoalsConstPtr &goals_msg)
{
  goals = goals_msg->list;
}

bool getGoal(unsigned int id, geometry_msgs::PoseStamped &pose)
{
  for (const auto goal : goals)
  {
    if (goal.destination_id == id)
    {
      pose = goal.pose;
      return true;
    }
  }

  return false;
}


int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "single_goal");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Subscribe to paths
  ros::Subscriber subscriber = node.subscribe("/goals", 1000, &goalsCallback);

  ros::Publisher goal_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);



  unsigned int destination_id = 3;
  geometry_msgs::PoseStamped current_goal;

  // Wait for path to become available
  ros::Rate wait_rate(10.0);
  while (ros::ok() && getGoal(destination_id, current_goal) == false)
  {
    ros::spinOnce();
    wait_rate.sleep();
  }

  goal_pub.publish(current_goal);

/*
  // Generate action message
  move_base_msgs::MoveBaseActionGoal action;

  actionlib_msgs::GoalID goal_id;
  goal_id.id = destination_id;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = current_goal;

  action.header.stamp = ros::Time::now();
  action.goal_id = goal_id;
  action.goal = goal;

  goal_pub.publish(action);
*/


  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
