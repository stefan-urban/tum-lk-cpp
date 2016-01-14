#pragma once

#include "TurtleBot.h"
#include "StateManager.h"

/**
 * Class for controlling the robot's movement.
 */
class Driver
{
public:

  /**
   * Move the robot to this waypoint. Note that it is assumed that there is no
   * obstacle between the robot's current position and the destination.
   */
  void gotoWaypoint();

  /**
   * Drive around while avoiding obstacles. Use this when there is no waypoint
   * for the next aruco marker available.
   */
  void performRandomWalk();

  /**
   * Update the FSM. Call this once every iteration of the main loop.
   */
  void tick();



private:

  /**
   * The FSM that controls the robot's behaviour.
   */
  StateManager stateManager;

  /**
   * Interface to the robot's current position and for movement orders.
   */
  TurtleBot turtleBot;
};
