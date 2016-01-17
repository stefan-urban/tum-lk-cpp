#pragma once

#include "TurtleBot.h"
#include "StateManager.h"

#include <pathfinder/Path.h>

/**
 * Class for controlling the robot's movement.
 */
class Driver
{
public:
  Driver();

  /**
   * Move the robot to this Marker. Note that it is assumed that there is no
   * obstacle between the robot's current position and the destination. Will do
   * nothing if the path is not known.
   * @param id: aruco id between 0 and 7
   */
  void gotoMarker(int id);

  /**
   * Drive around while avoiding obstacles. Use this when there is no waypoint
   * for the next aruco marker available.
   */
  bool performRandomWalk();

  /**
   * Stops any movement of the robot. This activates the state StateID::IDLE.
   */
  void stopRobot();

  /**
   * Update the FSM. Call this once every iteration of the main loop.
   */
  void tick();

private:

  /// ...
  void waypoint_callback(const pathfinder::PathConstPtr &pathmsg);

  /// ...
  ros::NodeHandle node;

  /// ...
  ros::Subscriber waypoint_subscriber;

  /// ...
  std::array<nav_msgs::Path, 8> paths;

  /**
   * The FSM that controls the robot's behaviour.
   */
  StateManager stateManager;

  /**
   * Interface to the robot's current position and for movement orders.
   */
  TurtleBot turtleBot;
};
