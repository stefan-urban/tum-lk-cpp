#pragma once

/**
 * Enum for identifying the states
 */
enum class StateID
{
  IDLE,                 ///< No movement
  RANDOM_WALK,          ///< Driving around while avoiding obstacles
  MOVING_TWIST,         ///< Moving according to a twist message
  MOVING_TO_LOCATION,   ///< Moving to the given location
  ROTATING,             ///< indicates ongoing rotation movement of the robot
  BUMPERHIT,            ///< executed when a bumper sensor signals a hit
};

/**
 * Base class for all states that will be used with the FSM.
 */
class State
{
public:
  State(StateID stateID)
    : stateID(stateID)
  {
  }

  /**
   * This function will be called when the state is used for the first time.
   */
  virtual void startup() {};

  /**
   * This function will be called once the state is the active state again.
   * Note that every saved data about the robot must be updated before it is
   * used.
   */
  virtual void resume() {};

  /**
   * Signal the state that it will be paused until the new state has finished
   * its work.
   */
  virtual void pause() {};

  /**
   * This function is called when the state is being stopped, albeit it has not
   * yet finished its work.
   */
  virtual void stop() {};

  /**
   * This function is called repeatedly.
   */
  virtual void tick() = 0;

  /**
   * Checks if this state has finished its work so it can be removed.
   * @return true if this state has finished its work
   */
   bool hasFinished()
   {
     return isFinished;
   }

  /**
   * Gets the ID of this state
   * @return the ID of the state
   */
  StateID getID()
  {
    return stateID;
  }

private:
  /**
  * Signals whether this state has completed its work. When this is set to true
  * before the end of the call to tick(), this state will be removed from the
  * FSM.
  */
  bool isFinished = false;

  /**
   * The type of the state. All types are specified in the Enum TurtleBotState.
   */
  StateID stateID;
};
