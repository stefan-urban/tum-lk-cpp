#include "Driver.h"

void Driver::tick()
{
  stateManager.tick();
}

void Driver::performRandomWalk()
{
  if(stateManager.currentState()->getID() != StateID::RANDOM_WALK)
  {

  }
}
