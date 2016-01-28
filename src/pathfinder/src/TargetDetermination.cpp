
#include "TargetDetermination.h"
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
    // @todo: Do not use aruco code position, instead try to reach pose 0.5m in
    //        front of it

    // Convert pose with covariance to
    geometry_msgs::PoseStamped pose;

    pose.header = marker.header;
    pose.pose = marker.pose.pose;

    pose.pose.position.z = 0;

    goals_[marker.id] = pose;
  }
}
