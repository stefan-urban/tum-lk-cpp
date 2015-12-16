#include "Rotator.h"
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

Rotator::Rotator()
: current_angle(0.0)
, initialized(false)
{
  // Publish turtlebot movement commands
  command_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  // Subscribe pose
  pose_sub = node.subscribe("/odom", 1, &Rotator::poseCallback, this);
}

void Rotator::resetRotation()
{
  initial_angle = current_angle;
}

float Rotator::getRotation()
{
  return std::abs(current_angle - initial_angle);
}

void Rotator::rotateAngle(float angle)
{
  ros::Rate loop_rate(10.0);

  // Wait for the first pose topic
  while (!initialized && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  resetRotation();

  int debounce = 0;

  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    ROS_INFO_STREAM("curr: " << getRotation() << " - goal: " << angle);

    // Stop if
    if (getRotation() >= angle)
    {
      if (debounce < 5 )
      {
        debounce++;
      }
      else
      {
        command_pub.publish(msg);
        break;
      }
    }
    else
    {
      msg.angular.z = ROTATION_SPEED;
    }

    command_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Rotator::poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  current_angle = tf::getYaw(odom->pose.pose.orientation);
  initialized = true;
}
