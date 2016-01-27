
#include "CommandLine.h"
#include <ros/ros.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <sstream>

CommandLine::CommandLine()
: run_(true)
{
  generatePromptMessage();
}

void CommandLine::generatePromptMessage()
{
  std::ostringstream oss;

  oss << CMD_PROMPT_HEADER;

  for (auto command : commands_)
  {
    oss << " - ";
    oss << command->getPromptString();
    oss << "\n";
  }

  oss << "\n";
  oss << CMD_PROMPT;

  prompt_msg_ = oss.str();
}

void CommandLine::start()
{
  run_ = true;
  unsigned int counter = 1;

  std::cout << prompt_msg_;

  while (run_ == true)
  {
    std::string cmd, ret_str;
    Parser parser(commands_);

    // Read from command line
    std::getline(std::cin, cmd);

    // Stop on exit command
    if (cmd == EXIT_CMD)
    {
      stop();
      break;
    }

    // Parse command
    ret_str = parser.process(cmd);

    std::cout << ret_str << "\n";

    // Output complete help text only each five commands
    if (counter++ % 5 == 0)
    {
      std::cout << prompt_msg_;
    }
    else
    {
      std::cout << CMD_PROMPT;
    }
  }
}

void CommandLine::stop()
{
  run_ = false;
  ros::shutdown();
}

void CommandLine::registerCmd(Cmd *command)
{
  commands_.push_back(command);

  generatePromptMessage();
}
