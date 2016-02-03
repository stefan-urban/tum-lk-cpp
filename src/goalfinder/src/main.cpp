#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <goalfinder/Goals.h>

#include "TargetDetermination.h"


geometry_msgs::PoseStamped current_pose;

void positionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  current_pose.header = pose->header;
  current_pose.pose = pose->pose.pose;
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "goalfinder");
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

  // Advertise goals topic
  ros::Publisher goals_pub = node.advertise<goalfinder::Goals>("/goals", 10);

  // Setup target determination
  TargetDetermination td;
  std::map<unsigned int, geometry_msgs::PoseStamped> goals;


  ROS_INFO("Startup finished!");

  // Loop
  ros::Rate loop_rate(1.0);

  while (ros::ok())
  {
    // Goals
    auto goals = td.getGoals();

    goalfinder::Goals goals_msg;

    for (const auto& goal : goals)
    {
      goalfinder::Goal goal_msg;

      goal_msg.destination_id = goal.first;
      goal_msg.pose = goal.second;

      goals_msg.list.push_back(goal_msg);
    }

    goals_pub.publish(goals_msg);

    // Publish goal frames to TF for debugging
    td.broadcastTf();

    ros::spinOnce();
    loop_rate.sleep();


  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
