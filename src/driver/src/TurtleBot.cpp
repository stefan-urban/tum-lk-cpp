#include "TurtleBot.h"

TurtleBot::TurtleBot()
{
  velcmd_publisher = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  pose_subscriber = node.subscribe("/odom", 1, &TurtleBot::poseCallback, this);
  //bumper_subscriber = node.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 1000, &TurtleBot::bumperCallback, this);
}

void TurtleBot::move(geometry_msgs::Point destination)
{

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

void TurtleBot::stop()
{
  move(geometry_msgs::Twist());
}

float TurtleBot::getTurnAngle(geometry_msgs::Point targetLocation)
{
  float targetAngle = atan2(current_position.x - targetLocation.x,
                            current_position.y - targetLocation.y);

  return targetAngle - current_rotation;                          
}

geometry_msgs::Point TurtleBot::getPosition()
{
  return current_position;
}

float TurtleBot::getRotation()
{
  return current_rotation;
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
  current_rotation = tf::getYaw(odom->pose.pose.orientation);
}

/*
void TurtleBot::bumperCallback(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg)
{
  if(msg->bumps_wheeldrops != 0)
  {
    // save the last action and initate a back off
    last_state = current_state;
    current_state = BUMPERHIT_BACK_OFF;

    // call move and drive backwards here, turning left or right, based on the laser data if available
  }
}
*/
