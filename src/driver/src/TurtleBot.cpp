#include "TurtleBot.h"

TurtleBot::TurtleBot()
{
  velcmd_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/driver", 10);

  pose_subscriber = node.subscribe("/odom", 1, &TurtleBot::poseCallback, this);
  //bumper_subscriber = node.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 1000, &TurtleBot::bumperCallback, this);
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

  // turnAngle is relative to our rotation
  targetAngle -= current_rotation;

  // clamp the value (can be negative!)
  if(targetAngle > 2*M_PI)
    targetAngle -= 2*M_PI;
  if(targetAngle < -2*M_PI)
    targetAngle += 2*M_PI;

  return targetAngle;
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

  current_position = odom->pose.pose.position;

  float new_angle = tf::getYaw(odom->pose.pose.orientation);
  if (new_angle < current_rotation)
  {
    new_angle += 2 * M_PI;
  }

  current_rotation = new_angle;
}
ros::NodeHandle *TurtleBot::getNode()
{
  return &node;
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
