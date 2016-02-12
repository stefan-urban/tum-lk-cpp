
#include <ros/ros.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>


/**
 * Provides all possible navigation goals, stores position, so even if they are
 * not visible by the sensor anymore, they are available
 */
class TargetDetermination
{
public:
  const std::string markers_topic = "/aruco_marker/markers";

  /**
   * Broadcasts the most recent position to all goals to TF
   */
  void broadcastTf();

  /**
   * Constructor: Subscribes to aruco topic and sets up parameters
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

  // Parameters
  float goal_distance_from_marker;

  /**
   * Takes every single markers, calculates a position in front of them and
   * stores it as movement goal. The distance between the marker and the goal is
   * defined by the parameter: goal_distance_from_marker
   */
  void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array);

};
