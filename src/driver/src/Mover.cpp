#include "Mover.h"
#include <geometry_msgs/Twist.h>

Mover::Mover()
{
  // Publish turtlebot movement commands
  command_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
}

void Mover::moveUntilCondition(bool (*function)())
{
  ros::Rate loop_rate(50.0);

  geometry_msgs::Twist msg;
  msg.linear.x = 0.2;

  while(ros::ok() && !function())
  {
    command_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Stop robot by sending msg again with clear values
  msg.linear.x = 0.0;
  command_pub.publish(msg);

}
