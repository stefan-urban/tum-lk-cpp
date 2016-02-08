
#include "TargetDetermination.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

TargetDetermination::TargetDetermination()
{
  markers_sub = node.subscribe(markers_topic.c_str(), 1, &TargetDetermination::markersCallback, this);

  // Try to get parameters
  if (!node.getParam("/goalfinder/goal_distance_from_marker", goal_distance_from_marker))
  {
    goal_distance_from_marker = 0.4;
  }

  ROS_INFO_STREAM("Goal distance will be " << goal_distance_from_marker << " meters.");
}

std::map<unsigned int, geometry_msgs::PoseStamped> TargetDetermination::getGoals()
{
  return goals_;
}

void TargetDetermination::markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array)
{
  std::vector<aruco_msgs::Marker> markers = marker_array->markers;

  for (const aruco_msgs::Marker &marker : markers)
  {
    // Create pose
    geometry_msgs::Pose msg_pose = marker.pose.pose;

    // Convert to TF
    tf::Pose tf_pose;
    tf::poseMsgToTF(msg_pose, tf_pose);

    // Translation alongside y
    tf::Transform trans;
    trans.setOrigin(tf::Vector3(0.0 , goal_distance_from_marker, 0.0));
    trans.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, 0.0) );

    tf_pose = tf_pose * trans;

    // Set height to zero
    tf::Vector3 pos = tf_pose.getOrigin();
    pos.setZ(0.0);
    tf_pose.setOrigin( pos );

    // Find yaw rate facing to marker
    float dx = marker.pose.pose.position.x - tf_pose.getOrigin().getX();
    float dy = marker.pose.pose.position.y - tf_pose.getOrigin().getY();

    float yaw = std::atan2(dy, dx);

    // Set rotation to face marker
    tf::Quaternion quad = tf_pose.getRotation(); // @todo
    quad.setRPY(0.0, 0.0, yaw);
    tf_pose.setRotation( quad );

    // Convert back to msg pose
    tf::poseTFToMsg(tf_pose, msg_pose);

    // Create stamped pose
    geometry_msgs::PoseStamped msg_posestamped;
    msg_posestamped.header = marker.header;
    msg_posestamped.pose = msg_pose;

    // And save
    goals_[marker.id] = msg_posestamped;
  }
}

void TargetDetermination::broadcastTf()
{
  static tf::TransformBroadcaster br;

  for (const auto& goal : goals_)
  {
    tf::Pose tf_pose;

    tf::poseMsgToTF(goal.second.pose, tf_pose);
    br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "map", "goal_" + std::to_string(goal.first)));

    ROS_INFO_STREAM("goal " << goal.first << " output");
  }
}
