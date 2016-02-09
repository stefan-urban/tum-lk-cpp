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

  // clamp the target angle to [0, 2pi]
  float targetAngle = turnAngle + startAngle;
  if(targetAngle < 0)
    targetAngle += 2*M_PI;
  if(targetAngle > 2*M_PI)
    targetAngle -= 2*M_PI;

  // is the rotation done?
	if(std::abs(currentAngle - targetAngle) <= 0.1f)
	{
		getTurtleBot()->stop();
		setFinished(true);
	} else {

    // turn into the direction that is faster to get to the target angle
    if(targetAngle > startAngle)
		  getTurtleBot()->move(0, 0.2);
    else
      getTurtleBot()->move(0, -0.2);
	}
}

std::string StateRotating::getDescription()
{
	float currentAngle = getTurtleBot()->getRotation();
	std::stringstream ss;
	ss << " Current angle: " << currentAngle << ", targetAngle: " << turnAngle;
	return ss.str();
}
