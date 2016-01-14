#include "StateManager.h"

#include "StateIdle.h"

StateManager::StateManager()
{
  push_state(std::make_shared<StateIdle>());
}
StateManager::~StateManager()
{
}

void StateManager::push_state(std::shared_ptr<State> state)
{
  states.push_back(state);
}
void StateManager::tick()
{
  if(!states.empty())
  {
    states.back()->tick();
  }
}
