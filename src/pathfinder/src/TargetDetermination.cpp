
#include "TargetDetermination.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


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
    // Define stamped pose
    geometry_msgs::PoseStamped pose;
    pose.pose = marker.pose.pose;

    // @todo: there has to be a function that does this in one line!

    // Find if already in storage
    std::map<unsigned int, geometry_msgs::PoseStamped>::iterator it = goals_.find(marker.id);

    if( it != goals_.end() )
    {
      // Found, replace
      goals_[marker.id] = pose;
    }
    else
    {
      // Not found, insert
      goals_.insert(std::pair<unsigned int, geometry_msgs::PoseStamped>(marker.id, pose));
    }
  }

  // debug
  debug_broadcast_tf();
}

void TargetDetermination::debug_broadcast_tf()
{
  static tf::TransformBroadcaster br;
  
  for (const auto& goal : goals_)
  {
    tf::Pose tf_pose;
    tf::poseMsgToTF(goal.second.pose, tf_pose);

    br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "map", "goal_" + std::to_string(goal.first)));
  }
}
