#include "Planner.h"
#include <nav_msgs/GetPlan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

Planner::Planner()
{
  // Subscribe to current pose
  pose_subscriber = node.subscribe("/odom", 1, &Planner::poseCallback, this);

  // Wait for move_base to start up
  while (!ros::service::waitForService(planner_service_name, ros::Duration(3.0)) || !ros::ok())
  {
    //ROS_INFO_STREAM("Waiting for service " << service_name << " to become available.");
  }

  // Setup service
  move_base_service = node.serviceClient<nav_msgs::GetPlan>(planner_service_name);

  if (!move_base_service)
  {
    ROS_FATAL("Move base does not provide path planner!");
  }

}

nav_msgs::Path Planner::makePlan(const geometry_msgs::PoseStamped &goal)
{
  // Check if service is still available
  if (!move_base_service)
  {
    ROS_FATAL("Move base does not provide path planner anymore!");
  }

  // Prepare request
  nav_msgs::GetPlan srv;

  // Current position is start point
  srv.request.start = current_pose_;
  srv.request.start.header.frame_id = frame_id;

  // Goal is destination
  srv.request.goal = goal;
  srv.request.goal.header.frame_id = frame_id;

  // Call service
  move_base_service.call(srv);


  return srv.response.plan;
}

void Planner::poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  current_pose_.header = odom->header;
  current_pose_.pose = odom->pose.pose;
}

void Planner::debug_broadcast_tf(unsigned int id, nav_msgs::Path path)
{
  static tf::TransformBroadcaster br;

  unsigned int counter = 0;

  for (const auto& goal : path.poses)
  {
    tf::Pose tf_pose;
    tf::poseMsgToTF(goal.pose, tf_pose);

    br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "map", "path" + std::to_string(counter++) + "_" + std::to_string(id)));
  }
}
