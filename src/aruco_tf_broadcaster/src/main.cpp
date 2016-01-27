#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <string>
#include <map>
#include <vector>


std::map<unsigned int, geometry_msgs::PoseStamped> markers_;

void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array)
{
  std::vector<aruco_msgs::Marker> markers = marker_array->markers;

  for (const aruco_msgs::Marker &marker : markers)
  {
    // Define stamped pose
    geometry_msgs::PoseStamped pose;
    pose.pose = marker.pose.pose;

    // @todo: there has to be a function that does this in one line!

    // Find if already in storage
    std::map<unsigned int, geometry_msgs::PoseStamped>::iterator it = markers_.find(marker.id);

    if( it != markers_.end() )
    {
      // Found, replace
      markers_[marker.id] = pose;
    }
    else
    {
      // Not found, insert
      markers_.insert(std::pair<unsigned int, geometry_msgs::PoseStamped>(marker.id, pose));
    }
  }
}

void broadcastTf(const ros::TimerEvent&)
{
  static tf::TransformBroadcaster br;

  for (const auto &marker : markers_)
  {
    tf::Pose tf_pose;
    tf::poseMsgToTF(marker.second.pose, tf_pose);

    br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "markers_link", "marker_" + std::to_string(marker.first)));
  }
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "aruco_tf_broadcaster_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();

  // Subscribe all aruco markers
  ros::Subscriber sub = node.subscribe("/aruco_marker/markers", 10, &markersCallback);

  // Send transformations every 100 ms
  ros::Timer timer = node.createTimer(ros::Duration(0.1), &broadcastTf, false);

  // Loop
  ros::spin();

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
