#include "TurtleBot.h"

TurtleBot::TurtleBot()
{
  velcmd_publisher = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  ros::Subscriber pose_subscriber = node.subscribe("/odom", 1, &TurtleBot::poseCallback, this);
}

void TurtleBot::move(geometry_msgs::Point destination)
{
  current_position;
}

void TurtleBot::move(geometry_msgs::Twist twist)
{
  velcmd_publisher.publish(twist);
}

void TurtleBot::move(float linear_speed, float angular_speed)
{
  geometry_msgs::Twist twist;

  twist.linear.x = linear_speed;
  twist.angular.z = angular_speed;

  move(twist);
}

void TurtleBot::poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  tf::Transform transform;

  tf::Quaternion quad;
  tf::quaternionMsgToTF(odom->pose.pose.orientation, quad);

  // Robot is center of universe
  transform.setOrigin(tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0));
  transform.setRotation(quad);

  tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtlebot_base"));

  current_position = odom->pose.pose.position;
}
