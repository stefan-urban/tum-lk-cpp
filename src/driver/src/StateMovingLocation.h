#pragma once

#include "State.h"

class StateMovingLocation : public State
{
public:
	StateMovingLocation(std::shared_ptr<TurtleBot> turtleBot, geometry_msgs::Point targetLocation, bool relative = false);

	virtual void tick();

	virtual std::string getDescription();

private:
	geometry_msgs::Point targetLocation;
};
