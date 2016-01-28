
#include <ros/ros.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>


/**
 * Provides all possible navigation goals, stores position even if they are not
 * visible by the sensor anymore
 */
class TargetDetermination
{
public:
  const std::string markers_topic = "/aruco_marker/markers";
  const float goal_distance_from_marker = 0.3;

  /**
   * Constructor: Subscribes to aruco topic
   */
  TargetDetermination();

  /**
   * Provides a list of all available goals
   */
  std::map<unsigned int, geometry_msgs::PoseStamped> getGoals();

private:
  ros::NodeHandle node;
  std::map<unsigned int, geometry_msgs::PoseStamped> goals_;

  ros::Subscriber markers_sub;
  void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array);
};
