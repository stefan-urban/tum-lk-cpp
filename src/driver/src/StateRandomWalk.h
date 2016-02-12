#pragma once

#include "State.h"
#include "sensor_msgs/LaserScan.h"

/**
 * Performs random walk (driving around while avoiding obstacles) on the
 * TurtleBot.
 * Note: this state will never finish.
 */
class StateRandomWalk : public State
{
public:
  StateRandomWalk(std::shared_ptr<TurtleBot> turtleBot);

  virtual void tick();

  constexpr static double MIN_SCAN_ANGLE_RAD = -25.0/180*M_PI;
	constexpr static double MAX_SCAN_ANGLE_RAD = +25.0/180*M_PI;
  constexpr static float TURN_SPEED = 0.2f;
  constexpr static float MOVEMENT_SPEED = 0.2f;
  constexpr static float MIN_PROXIMITY_RANGE_M = 0.7;

private:
  void scanCallback(const	sensor_msgs::LaserScan::ConstPtr& scan);

  bool doMove = true;
  int angleFactor = 1;
};
