
#include "TargetDetermination.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

TargetDetermination::TargetDetermination()
{
  markers_sub = node.subscribe(markers_topic.c_str(), 1, &TargetDetermination::markersCallback, this);
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

    // We only do 2D, so omit z coordinate
    msg_pose.position.z = 0;

    // Convert to TF
    tf::Pose tf_pose;
    tf::poseMsgToTF(msg_pose, tf_pose);

    tf_pose.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));

    // First Rotation around y
    tf::Transform trans1;
    auto rot1 = tf::Quaternion();
    rot1.setRPY(0.0, M_PI/2, 0.0);
    trans1.setRotation(rot1);

    // Second rotation around z
    tf::Transform trans2;
    auto rot2 = tf::Quaternion();
    rot2.setRPY(0.0, 0.0, -1 * M_PI/2);
    trans2.setRotation(rot2);

    // Translation alongside x
    tf::Transform trans3;
    trans3.setOrigin(tf::Vector3(-1 * goal_distance_from_marker, 0.0, 0.0));
    trans3.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, 0.0) );

    // Apply transformations
    tf_pose = tf_pose * trans1 * trans2 * trans3;

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
