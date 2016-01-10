#include "TurtleBot.h"

TurtleBot::TurtleBot()
  : current_rotation(0.0f), current_state(TurtleBotState::IDLE), last_state(TurtleBotState::IDLE)
{
  velcmd_publisher = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  pose_subscriber = node.subscribe("/odom", 1, &TurtleBot::poseCallback, this);
  bumper_subscriber = node.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 1000, &TurtleBot::bumperCallback, this);
}

void TurtleBot::move(geometry_msgs::Point destination)
{
  current_position = getPosition();

  current_state = MOVING_TO_LOCATION;
}

void TurtleBot::move(geometry_msgs::Twist twist)
{
  current_position = getPosition();
  current_state = MOVING_TWIST;
  
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
    {
      // Keep this state until the specified marker has been found.
      // Move forward until either a bumper has been hit or the laser scan
      // has found an obstacle in the way.

      // Some dummy code for driving forward
      geometry_msgs::Twist movement_message;
      movement_message.linear.x = 1;

      velcmd_publisher.publish(movement_message);


      break;
    }

    case TurtleBotState::MOVING_TO_LOCATION:
    {
      // Keep this state until the location has been reached.
      float distanceSqr = (destination.x - current_position.x) * (destination.x - current_position.x) +
                          (destination.y - current_position.y) * (destination.y - current_position.y);
      if (distanceSqr < 1)
      {
        current_state = TurtleBotState::IDLE;
      } else {
        // continue movement, ie publish movement message
      }
      break;
    }

    case TurtleBotState::MOVING_TWIST:
      // If the robot didn't move for 2-3 calls the moving has completed (just like with the wall detection in the homework)
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
  current_rotation = tf::getYaw(odom->pose.pose.orientation);
}

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