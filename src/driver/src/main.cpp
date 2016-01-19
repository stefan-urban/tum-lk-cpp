#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "Utility.h"
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

  /*while(ros::ok())
  {

    driver.tick();

    ros::spinOnce();
    loop_rate.sleep();
  }*/
  // Go from AruCo code #0 to #7

  for (int code_id = 0; code_id <= 7; code_id++)
  {
    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
      // Look if there is a path available

        // If yes, go to next waypoint

        // If not, perform random walk

      driver.tick();
    }
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
