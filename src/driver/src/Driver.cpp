#include "Driver.h"
#include "StateRandomWalk.h"
#include <iostream>

Driver::Driver()
{
  //waypoint_subscriber = node.subscribe("/path_waypoints", 1, &Driver::waypoint_callback, this);
}

void Driver::gotoMarker(int id)
{
  
}

bool Driver::pathAvailable(int id)
{
  return false;
}

bool Driver::performRandomWalk()
{
  if(stateManager.currentState()->getID() != StateID::IDLE)
  {
    // Already performing some type of movement, stop the robot first
    return false;
  }

  stateManager.push_state(std::make_shared<StateRandomWalk>());
  return true;
}

void Driver::stopRobot()
{
  stateManager.clear_states();
}

void Driver::tick()
{
  stateManager.tick();
}

void Driver::waypoint_callback(const pathfinder::PathConstPtr &pathmsg)
{
  int id = pathmsg->destination_id;
  if(id > 7)
    return;

  // @todo: determine better way to check for a updated path
  if(paths[id].header.seq > pathmsg->path.header.seq)
    paths[id] = pathmsg->path;
}
