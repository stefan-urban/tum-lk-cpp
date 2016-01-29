#pragma once

#include "Parser.h"
#include "Cmd.h"

#include <string>
#include <vector>

class CommandLine
{
public:
  const std::string CMD_PROMPT_HEADER = "\nTurtlebot Command Line Interface\n================================\n\nCommands:\n - exit\t\tstop cli\n";
  const std::string CMD_PROMPT = " > ";
  const std::string EXIT_CMD = "exit";

  CommandLine();
  void start();
  void stop();

  void registerCmd(Cmd *command);

private:
  bool run_;
  
  std::vector<Cmd*> commands_;

  void generatePromptMessage();
  std::string prompt_msg_;
};
