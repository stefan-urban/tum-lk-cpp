#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "Driver.h"


int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "driver");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  Driver driver;

  int aruco_id = 0;

  // Go from AruCo code #0 to #7
  for (int arucoID = 0; arucoID <= 7; arucoID++)
  {
    ros::Rate loop_rate(10.0);

    while (ros::ok() && !driver.isMarkerReached(arucoID))
    {
      // if the path to arucoID is not available, perform random walk (ie drive
      // around while avoiding obstacles)
      if(driver.pathAvailable(arucoID))
      {
        driver.gotoMarker(arucoID);
      } else {
        driver.performRandomWalk();
      }

      driver.tick();

      ros::spinOnce();
      ROS_INFO_STREAM("Current state: " << driver.getStateDescription());
    }
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
