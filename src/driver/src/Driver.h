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
   * Check if a path to the given aruco marker is available.
   * @param id: ID of the aruco marker
   * @return true if the path to the marker with the given id is available,
   *         false otherwise
   */
  bool pathAvailable(int id);

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

  void followPath(nav_msgs::Path path);

  std::string getStateDescription()
  {
    return stateManager.getStateDescription();
  }

private:

  /// ...
  void waypoint_callback(const pathfinder::PathConstPtr &pathmsg);

  geometry_msgs::Point getTargetFromWaypoints(const std::vector<geometry_msgs::Point> &path);

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
  std::shared_ptr<TurtleBot> turtleBot;

  const int PATH_INTERP_MAX_WAYPOINTS = 10;
  const float PATH_INTERP_MAX_ERROR = 1.0f;
};
