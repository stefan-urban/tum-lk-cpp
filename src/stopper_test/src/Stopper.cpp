
#include "Stopper.h"
#include <geometry_msgs/Twist.h>

Stopper::Stopper()
: keep_moving(true)
{
  // Publish turtlebot movement commands
  command_pub = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);

  // Subscribe laser scan data
  laser_sub = node.subscribe("scan", 1, &Stopper::scanCallback, this);
}

void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Find the range minimum
  int min_idx = std::ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int max_idx = std::floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

  closest_range = scan->ranges[0];

  ROS_INFO_STREAM("min idx: " << min_idx);
  ROS_INFO_STREAM("max idx: " << max_idx);

  for (int i = min_idx + 1; i <= max_idx; i++)
  {
    if (scan->ranges[i] < closest_range)
    {
      closest_range = scan->ranges[i];
    }
  }

  ROS_INFO_STREAM("closest range: " << closest_range);

  if (closest_range < MIN_PROXIMITY_RANGE_M)
  {
    keep_moving = false;
  }
}

void Stopper::moveForward()
{
  geometry_msgs::Twist msg;

  if (keep_moving)
  {
    // Set to default speed
    msg.linear.x = FORWARD_SPEED_MPS;
  }

  // Send to turtlebot
  command_pub.publish(msg);
}

void Stopper::startMoving()
{
  keep_moving = true;

  ros::Rate loop_rate(50.0);
  ROS_INFO("Start moving turtlebot!");

  while (ros::ok())
  {
    moveForward();

    ros::spinOnce();
    loop_rate.sleep();

    if (!keep_moving)
    {
      ROS_INFO("Stop moving!");
      break;
    }
  }
}
