#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <map>
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <pathfinder/Path.h>


geometry_msgs::Pose current_pose;

void poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  current_pose = odom->pose.pose;
}

std::vector<geometry_msgs::PoseStamped> marker_poses(8);

void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array)
{
  std::vector<aruco_msgs::Marker> markers = marker_array->markers;

  for (aruco_msgs::Marker &marker : markers)
  {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.pose = marker.pose.pose;

    marker_poses[marker.id] = pose;
  }
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  tf::TransformBroadcaster br;

  // Subscribe to current pose
  ros::Subscriber pose_subscriber = node.subscribe("/odom", 1, poseCallback);

  // Subcribe to AruCo markers
  ros::Subscriber markers_sub = node.subscribe("/aruco_marker_publisher/markers", 1, &markersCallback);

  // Publisher for available paths
  ros::Publisher path_pub = node.advertise<pathfinder::Path>("/paths", 10);

  // Service for path planning
  std::string service_name = "/move_base/make_plan";

  while (!ros::service::waitForService(service_name, ros::Duration(5.0)))
  {
    //ROS_INFO_STREAM("Waiting for service " << service_name << " to become available.");
  }

  ros::ServiceClient planner_client = node.serviceClient<nav_msgs::GetPlan>(service_name);

  if (!planner_client)
  {
    ROS_FATAL("Planner not available!");
  }


  // Constantly determine paths to all known AruCo codes
  ros::Rate loop_rate(1.0);

  while (ros::ok())
  {
    unsigned int counter = 0;

    for (auto destination = marker_poses.begin(); destination != marker_poses.end() ; ++destination, ++counter)
    {
      // Check if AruCo code is even available
      if (destination->header.stamp.sec == 0)
      {
        continue;
      }

      // Check if information is good or outdated
      ros::Duration age = ros::Time::now() - destination->header.stamp;

      if (age.sec > 10)
      {
        // Delete outdated information
        marker_poses.erase(marker_poses.begin() + counter);

        ROS_INFO_STREAM("Dropped goal_" << counter);
        continue;
      }

      // Calculate waypoints on path
      pathfinder::Path path;

      // Request plan from move base
      if (!planner_client)
      {
        ROS_FATAL("Planner service failed");
        return -1;
      }

      nav_msgs::GetPlan request;

      // Current position is start point
      request.request.start.header.frame_id = "map";
      request.request.start.pose = current_pose;

      request.request.goal.header.frame_id = "map";
      //request.request.goal.pose = destination->pose;
      request.request.goal.pose = current_pose;

      // todo use tf functions
      // robot x corresponds to x of aruco coordinates
      // robot y corresponds to z of aruco coordinates
      request.request.goal.pose.position.x += destination->pose.position.x;
      request.request.goal.pose.position.y += destination->pose.position.z;

      request.request.tolerance = 0;

      // Call service
      planner_client.call(request);

      // Publish
      path.path = request.response.plan;
      path_pub.publish(path);

      tf::Transform transform;
      transform.setOrigin( tf::Vector3(request.request.goal.pose.position.x, request.request.goal.pose.position.y, request.request.goal.pose.position.z) );
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);

      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "goal_" + std::to_string(counter)));

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
