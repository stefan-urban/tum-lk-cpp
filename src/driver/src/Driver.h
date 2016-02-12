#pragma once

#include "TurtleBot.h"
#include "StateManager.h"

#include <pathfinder/Path.h>

/**
 * Abstraction layer for the state-stack (given by the class StateManager).
 * This class allows higher-level control of the robot, such as providing an id
 * the robot should search for.
 * This node needs the pathfinder to run and send paths to the found aruco
 * markers.
 * Note that this class and the state stack is currently not used and therefore
 * only roughly tested. It should however work (sort of). An example usage is
 * given in the main-function of this node. In pseudo-C-code this class could be
 * used as follows:
 *
 * Driver driver;
 * while( !driver.isMarkerReached(id) )
 * {
 *     if( driver.pathAvailable(id) )
 *         driver.gotoMarker(id);
 *     else
 *         driver.performRandomWalk();
 * }
 *
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
   * Returns true if the specified marker has been reached.
   * @param id: aruco id between 0 and 7
   */
  bool isMarkerReached(int id);

  /**
   * Check if a path to the given aruco marker is available.
   * @param id: ID of the aruco marker
   * @return true if the path to the marker with the given id is available,
   *         false otherwise
   */
  bool pathAvailable(int id);

  /**
   * Drive around while avoiding obstacles. Use this when there is no waypoint
   * for the next aruco marker available. This function can be called repeatedly
   * without causing chaos.
   * @return true if random walk could be started (or is already active), false
   *         otherwise
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

  /**
   * Tries to follow the path (if available!) to the given marker.
   * @param id: aruco id between 0 and 7
   */
  void followPath(int id);

  /**
   * Returns a printable description of the currently active state.
   * @return description of the currentl action
   */
  std::string getStateDescription()
  {
    return stateManager.getStateDescription();
  }

private:

  /**
   * Callback for waypoint (path) data
   */
  void waypoint_callback(const pathfinder::PathConstPtr &pathmsg);

  /**
   * Uses interpolation to determine the next target location for the given path
   */
  geometry_msgs::Point getTargetFromWaypoints(const std::vector<geometry_msgs::Point> &path);

  /**
   * Gets the distance to the last waypoint in the path to the given marker-id
   */
  float getDistanceToLastPosition(int id);

  /**
   * Handle to the node
   */
  ros::NodeHandle node;

  /**
   * Subscriber for waypoint (path) messages
   */
  ros::Subscriber waypoint_subscriber;

  /**
   * Paths to the aruco markers. If no path is available, the path in the
   * position of the array that corresponds to the searched for aruco marker has
   * not been updated in the last second.
   */
  std::array<nav_msgs::Path, 8> paths;

  /**
   * The FSM that controls the robot's behaviour.
   */
  StateManager stateManager;

  /**
   * Interface to the robot's current position and for movement orders.
   */
  std::shared_ptr<TurtleBot> turtleBot;

  /**
   * Temporary variable for the currently targeted aruco marker.
   */
  int currentID = -1;

  /**
   * Configuration for the interpolation step. Better don't change these..
   */
  const int PATH_INTERP_MAX_WAYPOINTS = 10;
  const float PATH_INTERP_MAX_ERROR = 1.0f;
};
