
#ifndef __PATHFINDER_H
#define __PATHFINDER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <obstacle_detection/Obstacle.h>
#include <obstacle_detection/ObstacleArray.h>

/**
 * Find a possible path from the robots position to a given destination
 */
class Pathfinder
{
public:

  /**
   * Sets up all publishers and subscribers
   */
  Pathfinder();

  /**
   * Find path
   */
  std::vector<geometry_msgs::Point> findPathTo(geometry_msgs::Point destination);

private:
  ros::NodeHandle node;
  ros::Subscriber obstacles_sub;
  void obstaclesCallback(const obstacle_detection::ObstacleArrayConstPtr& obstacle);
};

#endif /* __PATHFINDER_H */
