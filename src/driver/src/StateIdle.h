#pragma once

#include "State.h"

/**
 * Does nothing.
 */
class StateIdle : public State
{
public:
  StateIdle();

  void tick();

private:

};
