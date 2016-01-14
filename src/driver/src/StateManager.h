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
   * @return false if the new state could not be set (this will be the case
   *         when a state with id StateID::BUMPERHIT is currently active;
   *         simply try again on the next tick), true otherwise
   */
  bool push_state(std::shared_ptr<State> state);

  /**
   * Pop the current state. For the next state on the stack, resume() will be
   * called. When there is be no state left, a new idle state will be pushed
   * onto the stack.
   */
  void pop_state();

  /**
   * Update the logic of the FSM and calls tick() for the currently active
   * state.
   */
  void tick();

  /**
   * Returns the currently active state.
   * @returns currently active state
   */
  std::shared_ptr<State> currentState();

private:
  std::vector<std::shared_ptr<State>> states;

};
