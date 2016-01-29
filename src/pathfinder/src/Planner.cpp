#include "Planner.h"
#include <nav_msgs/GetPlan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


Planner::Planner()
{
  // Wait for move_base to start up
  while (!ros::service::waitForService(planner_service_name, ros::Duration(3.0)) || !ros::ok())
  {
    //ROS_INFO_STREAM("Waiting for service " << service_name << " to become available.");
  }

  // Setup service
  move_base_service = node.serviceClient<nav_msgs::GetPlan>(planner_service_name);

  if (!move_base_service.isValid())
  {
    ROS_FATAL("Move base does not provide path planner!");
  }


  // Wait for move_base to start up
  while (!ros::service::waitForService(planner_service_name2, ros::Duration(3.0)) || !ros::ok())
  {
    //ROS_INFO_STREAM("Waiting for service " << service_name << " to become available.");
  }

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

  //ROS_INFO_STREAM("start: " << srv.request.start.pose.position.x << " - " << srv.request.start.pose.position.y << " - " << srv.request.start.pose.position.z);
  //ROS_INFO_STREAM("goal: " << srv.request.goal.pose.position.x << " - " << srv.request.goal.pose.position.y << " - " << srv.request.goal.pose.position.z);

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

  ROS_INFO_STREAM("Did receive " << srv.response.plan.poses.size() << " steps.");

  return srv.response.plan;
}

void Planner::debug_broadcast_tf(unsigned int id, nav_msgs::Path path)
{
  if (path.poses.size() < 2)
  {
    return;
  }

  static tf::TransformBroadcaster br;
  tf::Pose tf_pose;

  // First
  tf::poseMsgToTF(path.poses.front().pose, tf_pose);
  br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "map", "path_" + std::to_string(id) + "_start"));

  // Last
  tf::poseMsgToTF(path.poses.back().pose, tf_pose);
  br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "map", "path_" + std::to_string(id) + "_end"));
}
