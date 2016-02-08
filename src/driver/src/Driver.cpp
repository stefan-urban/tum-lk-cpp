#include "Driver.h"
#include "StateRandomWalk.h"
#include "StateMovingLocation.h"
#include "StateRotating.h"
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

  followPath(id);
}

bool Driver::isMarkerReached(int id)
{
  if(paths[id].poses.empty())
    return false;

  float distance = getDistanceToLastPosition(id);

  if(distance <= 0.1f)
    return true;

  return false;
}

float Driver::getDistanceToLastPosition(int id)
{
  geometry_msgs::Point goalPosition = paths[id].poses.back().pose.position;
  geometry_msgs::Point currentPosition = turtleBot->getPosition();
  float distance = std::sqrt((goalPosition.x-currentPosition.x)*(goalPosition.x-currentPosition.x) +
                             (goalPosition.y-currentPosition.y)*(goalPosition.y-currentPosition.y));
  return distance;
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
  currentID = -1;
  stateManager.clear_states();
}

void Driver::tick()
{
  stateManager.tick();
}

void Driver::followPath(int id)
{
  nav_msgs::Path path = paths[id];

  if(path.poses.empty())
    return;
  if(!stateManager.isIdle())
    return;

    currentID = id;

  // if we have nearly reached the last position drive directly to it and turn in its direction
  float distance = getDistanceToLastPosition(id);
  if(distance < 0.15f)
  {
    float angle = turtleBot->getRotation() - tf::getYaw(paths[id].poses.back().pose.orientation);
    stateManager.push_state(std::make_shared<StateRotating>(turtleBot, angle));
    stateManager.push_state(std::make_shared<StateMovingLocation>(turtleBot, paths[id].poses.back().pose.position));

    return;
  }

  // if not determine the next location
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
  {
    if(id == currentID)
      stopRobot();
    paths[id] = pathmsg->path;
  }
}
