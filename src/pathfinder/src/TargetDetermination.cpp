
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

    // Move goal to place in front of marker
    tf::Transform trans;
    trans.setOrigin( tf::Vector3(goal_distance_from_marker, 0.0, 0.0) );
    trans.setRotation( tf::Quaternion() );

    tf_pose *= trans;

    // Convert back to msg pose
    tf::poseTFToMsg(tf_pose, msg_pose);

    // Create stamped pose
    geometry_msgs::PoseStamped msg_posestamped;
    msg_posestamped.header = marker.header;
    msg_posestamped.pose = msg_pose;

    goals_[marker.id] = msg_posestamped;
  }
}
