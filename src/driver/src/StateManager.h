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

  void setTurtleBot(std::shared_ptr<TurtleBot> bot);

  /**
   * Switch to a new state. For the last active state, pause() will be called.
   * @param state: The new state.
   * @return false if the new state could not be set (this will be the case
   *         when a state with id StateID::BUMPERHIT is currently active;
   *         simply try again on the next tick), true otherwise
   */
  bool push_state(std::shared_ptr<State> state);

  /**
   * Stop the current state. For the next state on the stack, resume() will be
   * called, for the deleted state stop(). When there is be no state left, a new
   * idle state will be pushed onto the stack.
   */
  void pop_state();

  /**
   * Stops all states and activates idle mode. For every state on the stack,
   * stop() will be called.
   */
  void clear_states(bool pushIdleState = true);

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

  /**
   * Returns a short printable description of the currently active state.
   */
  std::string getStateDescription();

  /**
   * Returns true if the robot is in idle mode, i.e. he is ready for new commands.
   */
  bool isIdle();

private:
  std::vector<std::shared_ptr<State>> states;

  std::shared_ptr<TurtleBot> turtleBot;

};
