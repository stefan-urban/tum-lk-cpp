#include "Pathfinder.h"

Pathfinder::Pathfinder()
{
  // todo: obstacle callback
  obstacles_sub = node.subscribe("/obstacles", 1, &Pathfinder::obstaclesCallback, this);
}

std::vector<geometry_msgs::Point> Pathfinder::findPathTo(geometry_msgs::Point destination)
{
  std::vector<geometry_msgs::Point> waypoints;

  waypoints.push_back(destination);

  return waypoints;
}

void Pathfinder::obstaclesCallback(const obstacle_detection::ObstacleArrayConstPtr& obstacle_array)
{
  obstacles = obstacle_array->obstacles;

  for (obstacle_detection::Obstacle &obstacle : obstacles)
  {
    // Search if Unique ID is already used
    ROS_INFO_STREAM("Received obstacle #" << obstacle.unique_id << " at " << obstacle.pose.position.x << "-" << obstacle.pose.position.y);
  }
}
