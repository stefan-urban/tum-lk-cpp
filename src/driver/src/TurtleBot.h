#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
//#include <turtlebot_node/TurtlebotSensorState.h>
#include <string>


/**
 * Abstraction of the robots position and movement
 */
class TurtleBot
{
public:

  const std::string cmd_vel_topic = "/mobile_base/commands/velocity";

  /**
   * Advertises the movement command and provide tf
   */
  TurtleBot();

  /**
   * Move to a point in free space
   */
  void move(geometry_msgs::Point destination);

  /**
   * Move according to Twist
   */
  void move(geometry_msgs::Twist twist);

  /**
   * 2D movement
   * @param linear_speed: Movement in x direction, use negative value to
   *                      go backwards
   * @param angular_speed: Rotation around z axis clock-wise, use negative
   *                       values to rotate counter clock-wise
   */
  void move(float linear_speed, float angular_speed);

  /**
   * Returns robots position in relation to start point
   */
  geometry_msgs::Vector3 getPosition();


  /**
   + Returns the angle the turtle bot has to turn in order to face the target
   * location.
   */
  float getTurnAngle(geometry_msgs::Point targetLocation);

private:
  // ...
  ros::NodeHandle node;

  // ...
  ros::Publisher velcmd_publisher;

  // ...
  ros::Subscriber pose_subscriber;

  /**
   * Subscriber for bumper state changes
   */
  ros::Subscriber bumper_subscriber;

  /**
   * Callback for current robot pose
   */
  void poseCallback(const nav_msgs::Odometry::ConstPtr& odom);

  /**
   * Callback for bumper state
   */
  //void bumperCallback(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg);

  /**
   * Current position relative to starting point
   *    +x : Backwards
   *    -x : Forward
   *    +y : Right
   *    -y : Left
   *     z : always 0 because of 2D navigation
   */
  geometry_msgs::Point current_position;

  /**
   *
   */
  float current_rotation = 0.f;

  // ...
  tf::TransformBroadcaster tf_broadcaster;
};
