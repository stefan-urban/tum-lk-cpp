#pragma once

#include <vector>
#include <memory>

#include "State.h"

/**
 * A very simple "FSM" for use with the TurtleBot. This is not a general purpose
 * implementation of an FSM but rather a stack of states.
 * The transition between states are handled manually
 * by the class Driver.
 * When no other state is active, TurtleBotState::IDLE will be executed.
 */
class StateManager
{
public:
  StateManager();
  ~StateManager();

  /**
   * Switch to a new state. For the last active state, pause() will be called.
   * @param state: The new state.
   */
  void push_state(std::shared_ptr<State> state);

  /**
   * Pop the current state. For the next state on the stack, resume() will be
   * called. When there should be no state left, TurtleBotState::IDLE will be
   * pushed onto the stack.
   */
  void pop_state();

  /**
   * Update the logic of the FSM and calls tick() for the currently active
   * state.
   */
  void tick();

private:
  std::vector<std::shared_ptr<State>> states;

};
