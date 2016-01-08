#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <vector>

#include <pathfinder/Path.h>

#include "Pathfinder.h"


std::vector<aruco_msgs::Marker> markers;
std::vector<geometry_msgs::PointStamped> marker_positions(8);

void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array)
{
  markers = marker_array->markers;

  for (aruco_msgs::Marker &marker : markers)
  {
    geometry_msgs::PointStamped point;

    point.header.stamp = ros::Time::now();
    point.point = marker.pose.pose.position;

    marker_positions[marker.id] = point;
  }
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Subcribe markers
  ros::Subscriber markers_sub = node.subscribe("/aruco_marker_publisher/markers", 1, &markersCallback);

  // Publisher for paths
  ros::Publisher path_pub = node.advertise<pathfinder::Path>("/paths", 10);

  // Constantly determine paths to all known AruCo codes
  ros::Rate loop_rate(1.0);


  Pathfinder pathfinder;

  while (ros::ok())
  {
    unsigned int counter = 0;

//    ROS_INFO("-----------------------------------------");

    for (auto destination = marker_positions.begin(); destination != marker_positions.end() ; ++destination, ++counter)
    {
      // Check if element is available
      if (destination->header.stamp.sec == 0)
      {
//        ROS_INFO_STREAM("ID #" << counter << " not set.");
        continue;
      }

      // Check if information is good or outdated
      ros::Duration age = ros::Time::now() - destination->header.stamp;

      if (age.sec > 4)
      {
        marker_positions.erase(marker_positions.begin() + counter);

//        ROS_INFO_STREAM("ID #" << counter << " unset.");
        continue;
      }

//      ROS_INFO_STREAM("ID #" << counter << " was set " << age.sec << " seconds ago.");

      // Calculate waypoints on path
      pathfinder::Path path;

      path.header.stamp = ros::Time::now();
      path.destination_id = counter;
      path.waypoints = pathfinder.findPathTo(destination->point);

      path_pub.publish(path);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
