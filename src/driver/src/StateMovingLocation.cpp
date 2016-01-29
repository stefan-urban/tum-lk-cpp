#include "StateMovingLocation.h"

#include <sstream>

StateMovingLocation::StateMovingLocation(std::shared_ptr<TurtleBot> turtleBot, geometry_msgs::Point targetLocation)
  : State(StateID::MOVING_TO_LOCATION, turtleBot), targetLocation(targetLocation)
{
}

void StateMovingLocation::tick()
{
	geometry_msgs::Point currentLocation = getTurtleBot()->getPosition();
	float distSqr = (currentLocation.x - targetLocation.x) * (currentLocation.x - targetLocation.x)
				  + (currentLocation.y - targetLocation.y) * (currentLocation.y - targetLocation.y);
	if(distSqr <= 0.5f)
	{
		getTurtleBot()->stop();
		setFinished(true);
	} else {
		float turnAngle = getTurtleBot()->getTurnAngle(targetLocation);
		turnAngle /= std::abs(turnAngle);

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
