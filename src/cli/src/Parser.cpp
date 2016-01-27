
#include "Parser.h"

Parser::Parser(std::vector<Cmd*> commands)
: commands_(commands)
{

}

std::string Parser::process(std::string cmd)
{
  // Iterate through all available commands and try to match cmd string
  for (auto command : commands_)
  {
    if (command->match(cmd))
    {
      return command->run();
    }
  }

  return std::string("unknown command");
}
