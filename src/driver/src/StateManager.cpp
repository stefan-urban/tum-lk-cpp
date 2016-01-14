#include "StateManager.h"

#include "StateIdle.h"

StateManager::StateManager()
{
  push_state(std::make_shared<StateIdle>());
}

StateManager::~StateManager()
{
}

bool StateManager::push_state(std::shared_ptr<State> state)
{
  if(currentState()->getID() == StateID::BUMPERHIT)
  {
    // Do not overwrite the state when we are backing off from a bumper hit
    // event so we do not run into the obstacle again and have two BUMPERHIT
    // states on the stack
    return false;
  }

  states.push_back(state);
  return true;
}

void StateManager::tick()
{
  if(!states.empty())
  {
    states.back()->tick();
  }
}

std::shared_ptr<State> StateManager::currentState()
{
  if(states.empty())
  {
    push_state(std::make_shared<StateIdle>());
  }

  return states.back();
}
