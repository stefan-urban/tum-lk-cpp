#pragma once

#include "Cmd.h"

#include <string>
#include <vector>

class Parser
{
public:
  Parser(std::vector<Cmd*> commands);
  std::string process(std::string cmd);

private:
  std::vector<Cmd*> commands_;
};
