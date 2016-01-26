
#include "CommandLine.h"

CommandLine::CommandLine()
: run_(true)
{

}

void CommandLine::start()
{
  run_ = true;

  while (run_ == true);
}

void CommandLine::stop()
{
  run_ = false;
}
