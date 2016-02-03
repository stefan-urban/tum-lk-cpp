
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

  void broadcastTf();

  /**
   * Constructor: Subscribes to aruco topic
   */
  TargetDetermination();

  /**
   * Provides a list of all available goals
   */
  std::map<unsigned int, geometry_msgs::PoseStamped> getGoals();

  /**
   * Get a vector of goals
   */
  std::vector<geometry_msgs::PoseStamped> getGoalsVec();

private:
  ros::NodeHandle node;
  std::map<unsigned int, geometry_msgs::PoseStamped> goals_;

  ros::Subscriber markers_sub;
  void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array);

  float goal_distance_from_marker;


};
