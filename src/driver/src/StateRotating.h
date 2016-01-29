#pragma once

#include "State.h"

class StateRotating : public State
{
public:
  StateRotating(std::shared_ptr<TurtleBot> turtleBot, float angle);
  StateRotating(std::shared_ptr<TurtleBot> turtleBot, geometry_msgs::Point targetLocation);

  virtual void tick();

  virtual std::string getDescription();

private:
	float turnAngle;
	float startAngle;

};
