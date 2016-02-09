#include "StateRandomWalk.h"
#include <math.h>

StateRandomWalk::StateRandomWalk(std::shared_ptr<TurtleBot> turtleBot)
  : State(StateID::RANDOM_WALK, turtleBot)
{
  getTurtleBot()->getNode()->subscribe("scan", 1, &StateRandomWalk::scanCallback, this);
}

void StateRandomWalk::tick()
{
  if(doMove)
  {
    getTurtleBot()->move(MOVEMENT_SPEED, 0);
  } else {
    getTurtleBot()->move(0, TURN_SPEED * angleFactor);
  }
}

void StateRandomWalk::scanCallback(const	sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Find the range minimum
  int min_idx = std::abs(std::ceil(MIN_SCAN_ANGLE_RAD / scan->angle_increment));
  int max_idx = std::abs(std::floor(MAX_SCAN_ANGLE_RAD / scan->angle_increment));

  int scan_count = std::abs(2*M_PI / scan->angle_increment);

  float closest_range_right = scan->ranges[min_idx];
  float closest_range_left = scan->ranges[min_idx];

  // scan the right side
  for (int i = 1; i <= max_idx-1; i++)
  {
    if (scan->ranges[i] < closest_range_left)
    {
      closest_range_left = scan->ranges[i];
    }
  }
  // scan the left side
  for (int i = scan_count-min_idx; i <= scan_count-1 ; i++)
  {
    if (scan->ranges[i] < closest_range_right)
    {
      closest_range_right = scan->ranges[i];
    }
  }

  if (closest_range_right < MIN_PROXIMITY_RANGE_M)
  {
    angleFactor = +1;
    doMove = false;
  } else if (closest_range_left  < MIN_PROXIMITY_RANGE_M)
  {
    angleFactor = 1;
    doMove = false;
  } else {
    doMove = true;
  }
}
