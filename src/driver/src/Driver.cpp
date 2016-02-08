#include "Driver.h"
#include "StateRandomWalk.h"
#include "StateMovingLocation.h"
#include <iostream>

Driver::Driver()
{
  turtleBot = std::make_shared<TurtleBot>();
  stateManager.setTurtleBot(turtleBot);
  waypoint_subscriber = node.subscribe("/paths", 1, &Driver::waypoint_callback, this);
}

void Driver::gotoMarker(int id)
{
  if(!pathAvailable(id))
    return;

  if(stateManager.currentState()->getID() == StateID::RANDOM_WALK)
  {
    stateManager.clear_states();
    stopRobot();
  }

  followPath(paths[id]);
}

bool Driver::isMarkerReached(int id)
{
  return false;
}

bool Driver::pathAvailable(int id)
{
  return paths[id].poses.empty();
}

bool Driver::performRandomWalk()
{
  if(!stateManager.isIdle())
  {
    // Already performing random walk?
    if(stateManager.currentState()->getID() == StateID::RANDOM_WALK)
      return true;

    // Already performing some other type of movement, stop the robot first
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

void Driver::followPath(nav_msgs::Path path)
{
  if(path.poses.empty())
    return;
  if(!stateManager.isIdle())
    return;

  std::vector<geometry_msgs::Point> waypoints;
  for(int i = 0; i < path.poses.size(); i++)
  {
    waypoints.push_back(path.poses[i].pose.position);
  }

  geometry_msgs::Point dest = getTargetFromWaypoints(waypoints);

  stateManager.push_state(std::make_shared<StateMovingLocation>(turtleBot, dest));
}

geometry_msgs::Point Driver::getTargetFromWaypoints(const std::vector<geometry_msgs::Point> &path)
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
