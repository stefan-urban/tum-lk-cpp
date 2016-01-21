#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_ros/aruco_ros_utils.h>

void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array)
{
  static tf::TransformBroadcaster br;
  std::vector<aruco_msgs::Marker> markers = marker_array->markers;

  for (const aruco_msgs::Marker &marker : markers)
  {
    tf::Pose tf_pose;
    tf::poseMsgToTF(marker.pose.pose, tf_pose);

    br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "markers_link", "marker_" + marker.id));
  }
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "aruco_tf_broadcaster_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Subscribe all aruco markers
  ros::Subscriber sub = node.subscribe("/aruco_marker/markers", 10, &markersCallback);

  // Loop
  ros::spin();

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
