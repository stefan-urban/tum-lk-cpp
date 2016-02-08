#include "StateManager.h"

#include "StateIdle.h"

StateManager::StateManager()
{
  // when the stack is "empty", the robot is idling
  push_state(std::make_shared<StateIdle>());
}
StateManager::~StateManager()
{
  clear_states(false);
}

void StateManager::setTurtleBot(std::shared_ptr<TurtleBot> bot)
{
  turtleBot = bot;
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

  // signal the currently active state to pause
  states.back()->pause();

  // push the new state and initialize it
  states.push_back(state);
  states.back()->startup();

  return true;
}

void StateManager::pop_state()
{
  if(states.empty())
  {
    // the stack is empty already, push an idle state onto it
    push_state(std::make_shared<StateIdle>());
    return;
  }

  // stop the state and delete it
  states.back()->stop();
  states.pop_back();

  // if the stack is empty now, push an idle state onto it
  if(states.empty())
  {
    push_state(std::make_shared<StateIdle>());
  }
}

void StateManager::clear_states(bool pushIdleState)
{
  // call stop() for each state on the stack
  for(auto &state : states)
    state->stop();

  // clear the stack and push an idle state
  states.clear();

  if(pushIdleState)
    push_state(std::make_shared<StateIdle>());
}

void StateManager::tick()
{
  // make sure that the stack is not empty
  if(states.empty())
    push_state(std::make_shared<StateIdle>());

  states.back()->tick();

  // remove the last state when it has finished
  // note that StateID::IDLE will never finish
  if(states.back()->hasFinished())
  {
    states.pop_back();

    if(states.empty())
      push_state(std::make_shared<StateIdle>());
      
    states.back()->resume();
  }
}

std::shared_ptr<State> StateManager::currentState()
{
  if(states.empty())
  {
    // do not use push_state here
    states.push_back(std::make_shared<StateIdle>());
  }

  return states.back();
}

std::string StateManager::getStateDescription()
{
  return stateNames[states.back()->getID()] + states.back()->getDescription();
}

bool StateManager::isIdle()
{
  return states.back()->getID() == StateID::IDLE;
}
