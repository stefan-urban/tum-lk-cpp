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
   * This function is called repeatedly.
   */
  virtual void tick() = 0;

  /**
  * Signals whether this state has completed its work. When this is set to true
  * before the end of the call to tick(), this state will be removed from the
  * FSM.
  */
  bool isFinished = false;

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
   * The type of the state. All types are specified in the Enum TurtleBotState.
   */
  StateID stateID;
};
