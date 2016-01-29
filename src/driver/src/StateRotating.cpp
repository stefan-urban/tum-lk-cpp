#include "StateRotating.h"

#include <sstream>

StateRotating::StateRotating(std::shared_ptr<TurtleBot> turtleBot, float angle)
  : State(StateID::ROTATING, turtleBot)
{
	turnAngle = angle;
	startAngle = getTurtleBot()->getRotation();
}

StateRotating::StateRotating(std::shared_ptr<TurtleBot> turtleBot, geometry_msgs::Point targetLocation)
  : State(StateID::ROTATING, turtleBot)
{
	turnAngle = getTurtleBot()->getTurnAngle(targetLocation);
	startAngle = getTurtleBot()->getRotation();
}

void StateRotating::tick()
{
	float currentAngle = getTurtleBot()->getRotation();
	if(currentAngle >= turnAngle - startAngle)
	{
		getTurtleBot()->stop();
		setFinished(true);
	} else {
		getTurtleBot()->move(0, 0.5);
	}
}

std::string StateRotating::getDescription()
{
	float currentAngle = getTurtleBot()->getRotation();
	std::stringstream ss;
	ss << " Current angle: " << currentAngle << ", targetAngle: " << turnAngle;
	return ss.str();
}