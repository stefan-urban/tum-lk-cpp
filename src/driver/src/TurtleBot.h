
#ifndef __TURTLEBOT_H
#define __TURTLEBOT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>


/// Indicates the state of a TurtleBot.
enum class TurtleBotState
{
  IDLE, ///< no movement
  RANDOM_WALK, ///< searching for the specified marker while avoiding obstacles
  MOVING_TWIST, ///< Moving according to a twist message
  MOVING_TO_LOCATION, ///< moving to the given location
  ROTATING, ///< indicates ongoing rotation movement of the robot
  BUMPERHIT_BACK_OFF, ///< executed when a bumper sensor signals a hit
};

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
   * Returns the current state of the robot
   */
  TurtleBotState getState();

  /**
   * Sets the state of the robot
   * @param new_state: Specifies the new state for the robot.
   */
  void setState(TurtleBotState new_state);

  /**
   * @todo: Maybe move the FSM and all the logic for finding AruCo markers
   *        (ie searchMarker) to the main function? If not, we need to check the
   *        camera feed when a marker has been found and tell the robot about it
   */

  /**
   * Initiates the search for the AruCo marker with the given number
   * @param marker_number: The ID of the AruCo marker the robot will search for
   *                       by driving around (random walk) while avoiding
   *                       obstacles.
   */ 
  void searchMarker(int marker_number);

  /**
   * Executes one step of the finite state machine of the robot. Call this
   * repeatedly.
   * If the state is not modified by a call to setState, after completion all
   * movement related states except BUMPERHIT_BACK_OFF will transition to the
   * state IDLE. BUMPERHIT_BACK_OFF will continue with the last state that was
   * active before the hit occurred.
   */
  void tick();

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
  void bumperCallback(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg);

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
   * Current rotation of the robot
   */
  float current_rotation;

  /**
   * Destination for moving to a specific location
   */
  geometry_msgs::Point destination;

  // ...
  tf::TransformBroadcaster tf_broadcaster;

  /**
   * Current state of the robot
   */
  TurtleBotState current_state;

  /*
   * The last state after the recent state transition.
   * This is used to resume the last state before BUMPERHIT_BACK_OFF.
   */
  TurtleBotState last_state;
};

#endif /* __TURTLEBOT_H */
