#include "Planner.h"
#include <nav_msgs/GetPlan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


Planner::Planner()
{
  // Wait for move_base to start up
  while (!ros::service::waitForService(planner_service_name, ros::Duration(3.0)) || !ros::ok());

  // Setup service
  move_base_service = node.serviceClient<nav_msgs::GetPlan>(planner_service_name);

  if (!move_base_service.isValid())
  {
    ROS_FATAL("Move base does not provide path planner!");
  }


  // Wait for move_base to start up
  while (!ros::service::waitForService(planner_service_name2, ros::Duration(3.0)) || !ros::ok());

  // Setup service
  move_base_service2 = node.serviceClient<nav_msgs::GetPlan>(planner_service_name2);

  if (!move_base_service2.isValid())
  {
    ROS_FATAL("Move base does not provide path planner2!");
  }

}

nav_msgs::Path Planner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
  // Check if service is still available
  if (!move_base_service.isValid() || !move_base_service2.isValid())
  {
    ROS_FATAL("Move base does not provide path planner anymore!");
  }

  // Prepare request
  nav_msgs::GetPlan srv;

  srv.request.start = start;
  srv.request.goal = goal;

  // Brute force the plan out of move_base
  unsigned int max_tries = 6;

  while (srv.response.plan.poses.size() == 0 && max_tries-- > 0)
  {
    if (max_tries % 2)
    {
      // Call service
      move_base_service.call(srv);
    }
    else
    {
      // Call service
      move_base_service2.call(srv);
    }

    ros::Duration(0.1).sleep();
  }

  return srv.response.plan;
}
