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
#include <kobuki_msgs/Sound.h>
#include <std_msgs/Int32.h>

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

unsigned int destination_id;

void cliGoalSetCallback(const std_msgs::Int32& goalId)
{
  destination_id = goalId.data;
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "multiple_goals");
  ros::NodeHandle node;

  // Start ROS
  ros::start();

  // Subscribe to paths
  ros::Subscriber goals_sub = node.subscribe("/goals", 1000, &goalsCallback);
  ros::Subscriber pose_sub = node.subscribe("/acml_pose", 1000, &poseCallback);
  ros::Subscriber cli_goal_id_sub = node.subscribe("/cli/goals/set_id", 1000, &cliGoalSetCallback);

  // Publish velocity command for rotation
  std::string cmd_vel_topic_name;

  if (!node.getParam("rotation_cmd_vel_command", cmd_vel_topic_name))
  {
    cmd_vel_topic_name = "/mobile_base/commands/velocity";
  }

  ros::Publisher search_pub = node.advertise<geometry_msgs::Twist>(cmd_vel_topic_name.c_str(), 10);

  // Publisher for kobuki sound commands
  std::string sound_topic_name;

  if (!node.getParam("sound_command", sound_topic_name))
  {
    sound_topic_name = "/mobile_base/commands/sound";
  }

  ros::Publisher sound_pub = node.advertise<kobuki_msgs::Sound>(sound_topic_name.c_str(), 10);

  kobuki_msgs::Sound sound_msg;
  sound_msg.value = 0;



  // Just loop through all destination ids
  for (destination_id = 0; destination_id < 8; destination_id++)
  {
    ROS_INFO_STREAM("Find id " << destination_id << " now!");

    geometry_msgs::PoseStamped current_goal;

    // Wait for path to become available
    ros::Rate wait_rate(2.0);
    while (ros::ok() && getGoal(destination_id, current_goal) == false)
    {
      geometry_msgs::Twist rotate_cmd;
      rotate_cmd.angular.z = 0.5;

      search_pub.publish(rotate_cmd);

      ROS_INFO("Wait for goal, rotating.");
      ros::spinOnce();
      wait_rate.sleep();
    }

    // If Strg+C was pressed
    if (!ros::ok())
    {
      return 0;
    }

    // Make sure robot has stopped
    search_pub.publish(geometry_msgs::Twist());

    // Setup action client
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    bool finished_before_timeout;

    do {
      // Update Goal
      getGoal(destination_id, current_goal);

      // Send updated goal position
      goal.target_pose = current_goal;

      ROS_INFO("Send goal.");
      ac.sendGoal(goal);

      // Wait 2 seconds and check if we succeeded
      finished_before_timeout = ac.waitForResult(ros::Duration(2.0));

      ros::spinOnce();

    } while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);



    ROS_INFO_STREAM("Now at goal #" << destination_id);
    sound_pub.publish(sound_msg);
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
