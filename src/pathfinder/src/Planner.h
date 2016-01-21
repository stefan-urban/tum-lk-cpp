#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

/**
 * Connects to move_base and is able to obtain a path from the current position
 * to a given goal
 */
class Planner
{
public:
  const std::string planner_service_name = "/move_base/make_plan";
  const std::string frame_id = "map";

  /**
   * Constructor: Setup connection to move_base
   */
  Planner();

  /**
   * Make a plan to a given goal from the current position
   */
  nav_msgs::Path makePlan(const geometry_msgs::PoseStamped &goal);

private:
  ros::NodeHandle node;

  ros::Subscriber pose_subscriber;
  void poseCallback(const nav_msgs::Odometry::ConstPtr& odom);
  geometry_msgs::PoseStamped current_pose_;

  ros::ServiceClient move_base_service;
};
