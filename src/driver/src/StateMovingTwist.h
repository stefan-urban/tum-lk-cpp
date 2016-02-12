#pragma once

#include "State.h"

/**
 * Currently unused. Use StateMovingLocation instead.
 */
class StateMovingTwist : public State
{
public:
  StateMovingTwist();

  virtual void tick();

private:

};
