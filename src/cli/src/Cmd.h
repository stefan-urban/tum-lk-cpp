#pragma once

#include <string>

class Cmd
{
public:
  /**
   * Checks if command entered into command line is handled by this class. If
   * true is returned, the run function can be called
   */
  virtual bool match(std::string cmd) = 0;

  /**
   * Runs command with previously match command string
   */
  virtual std::string run() = 0;

  /**
   * Returns string that displays in help message
   */
  virtual std::string getPromptString() = 0;
};
