#include "StateMovingLocation.h"
#include "Utility.h"

#include <sstream>

StateMovingLocation::StateMovingLocation(std::shared_ptr<TurtleBot> turtleBot, geometry_msgs::Point targetLoc, bool relative)
  : State(StateID::MOVING_TO_LOCATION, turtleBot)
{
  if(relative)
  {
    geometry_msgs::Point currentLocation = getTurtleBot()->getPosition();
    targetLocation = targetLoc;
    targetLocation.x += currentLocation.x;
    targetLocation.y += currentLocation.y;
  }
  else
    targetLocation = targetLoc;
}

void StateMovingLocation::tick()
{
	geometry_msgs::Point currentLocation = getTurtleBot()->getPosition();
	float distSqr = (currentLocation.x - targetLocation.x) * (currentLocation.x - targetLocation.x)
                + (currentLocation.y - targetLocation.y) * (currentLocation.y - targetLocation.y);
	if(distSqr <= 0.5f)
	{
    // we arrived at our target location
		getTurtleBot()->stop();
		setFinished(true);
	} else {
		float turnAngle = getTurtleBot()->getTurnAngle(targetLocation);
    // determine the turn direction (left or right), i.e. the sign of the angle to the target
		int turnDirection = sgn(turnAngle);

		getTurtleBot()->move(1.0f, 0.1f * turnAngle);
	}
}

std::string StateMovingLocation::getDescription()
{
	geometry_msgs::Point currentLocation = getTurtleBot()->getPosition();
	std::stringstream ss;
	ss << " current Location: (" << currentLocation.x << "|" << currentLocation.y << "), target: (" << targetLocation.x << "|" << targetLocation.y << ")";
	return ss.str();
}
