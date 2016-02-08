#include "Driver.h"
#include "StateRandomWalk.h"
#include "StateMovingLocation.h"
#include <iostream>

Driver::Driver()
{
  turtleBot = std::make_shared<TurtleBot>();
  stateManager.setTurtleBot(turtleBot);
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
  if(!stateManager.isIdle())
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

void Driver::followPath(std::vector<geometry_msgs::Point> path)
{
  if(path.empty())
    return;
  if(!stateManager.isIdle())
    return;

  geometry_msgs::Point dest = getTargetFromPath(path);

  stateManager.push_state(std::make_shared<StateMovingLocation>(turtleBot, dest));
}

geometry_msgs::Point Driver::getTargetFromPath(const std::vector<geometry_msgs::Point> &path)
{
  float firstAngle = turtleBot->getTurnAngle(path.at(0));
  geometry_msgs::Point dest = path.front();

  int point_counter = 0;
  for(auto loc : path)
  {
    if(point_counter++ >= PATH_INTERP_MAX_WAYPOINTS)
      break;

    if(std::abs(turtleBot->getTurnAngle(loc) - firstAngle) >= PATH_INTERP_MAX_ERROR)
      break;

    dest = loc;
  }

  return dest;
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
