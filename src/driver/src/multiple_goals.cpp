#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <pathfinder/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <pathfinder/Goals.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



std::vector<pathfinder::Goal> goals;

void goalsCallback(const pathfinder::GoalsConstPtr &goals_msg)
{
  if (goals_msg->list.size() == 0)
  {
    return;
  }

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


geometry_msgs::Pose current_pose;

//void poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
  current_pose = pose->pose.pose;
  //current_pose = odom->pose.pose;
}

unsigned int current_goal_status = 0;

void statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& status)
{
  if (status->status_list.size() == 0)
  {
    return;
  }

  current_goal_status = status->status_list.back().status;
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "multiple_goals");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Subscribe to paths
  ros::Subscriber goals_sub = node.subscribe("/goals", 1000, &goalsCallback);
  //  ros::Subscriber pose_sub = node.subscribe("/odom", 1000, &poseCallback);
  ros::Subscriber pose_sub = node.subscribe("/acml_pose", 1000, &poseCallback);
  ros::Subscriber status_sub = node.subscribe("/move_base/status", 1000, &statusCallback);

  // For rotation
  ros::Publisher search_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  // Simple goal ublisher
  ros::Publisher simple_goal_pub = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

  for (unsigned int destination_id = 0; destination_id < 8; destination_id++)
  {
    ROS_INFO_STREAM("Find id " << destination_id);

    geometry_msgs::PoseStamped current_goal;

    // Wait for path to become available
    ros::Rate wait_rate(2.0);
    while (ros::ok() && getGoal(destination_id, current_goal) == false)
    {
      geometry_msgs::Twist rotate_cmd;
      rotate_cmd.angular.z = 0.5;

      search_pub.publish(rotate_cmd);

      ROS_INFO("Wait for goal");
      ros::spinOnce();
      wait_rate.sleep();
    }

    if (!ros::ok())
    {
      return 0;
    }

    // Stop Robot
    search_pub.publish(geometry_msgs::Twist());

/*
    float distance;
    ros::Rate travel_rate(0.2);

    auto start_time = ros::Time::now();
    ros::Duration duration;

    do {
      // Update Goal
      getGoal(destination_id, current_goal);

      simple_goal_pub.publish(current_goal);
      distance = std::hypot(current_goal.pose.position.x - current_pose.position.x, current_goal.pose.position.y - current_pose.position.y);

      ROS_INFO_STREAM(destination_id << " - status " << current_goal_status << " distance " << distance << " to: " << current_goal.pose.position.x << "-" << current_goal.pose.position.y);

      ros::spinOnce();
      travel_rate.sleep();

      duration = ros::Time::now() - start_time;
    }
    while((ros::ok() && current_goal_status != 3) || ((ros::ok() && duration.toSec() < 5)));

    current_goal_status = 0;
*/


    // Setup action client
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    float distance;

    ac.sendGoal(goal);

    ros::Rate travel_rate(20.0);
    do {
      // Update Goal
      getGoal(destination_id, current_goal);

      distance = std::hypot(current_goal.pose.position.x - current_pose.position.x, current_goal.pose.position.y - current_pose.position.y);

      // Send updated goal position
      goal.target_pose = current_goal;

      ROS_INFO("Send goal. Status: ");

      ros::spinOnce();
      travel_rate.sleep();

    } while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);



    ROS_INFO_STREAM("at " << destination_id);

  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
