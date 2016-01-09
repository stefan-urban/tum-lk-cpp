#include "TurtleBot.h"

TurtleBot::TurtleBot()
  : current_state(TurtleBotState::IDLE), last_state(TurtleBotState::IDLE)
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

TurtleBotState TurtleBot::getState()
{
  return current_state;
}

void TurtleBot::setState(TurtleBotState new_state)
{
  current_state = new_state;
}

void TurtleBot::tick()
{
  switch(current_state)
  {
    case TurtleBotState::IDLE:
      // Do nothing here.
      break;

    case TurtleBotState::RANDOM_WALK:
      // Keep this state until the specified marker has been found.
      break;

    case TurtleBotState::MOVING_TO_LOCATION:
      // Keep this state until the location has been reached.
      // Either measure when the robot has stopped (for calls to move() with twist-values or
      // linear and angular speed) or compare the current position to the destination.
      /// @todo: break this up into an extra state for movements via linear/angular speed and twist messages?
      break;

    case TurtleBotState::BUMPERHIT_BACK_OFF:
      // Move back a bit and turn to the left or right, depending on the angle of incident to the wall.
      break;

    default:
      current_state = TurtleBotState::IDLE;
  }
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
