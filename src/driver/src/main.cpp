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

      if(driver.pathAvailable(code_id))
      {
        driver.gotoMarker(code_id);
      } else {
        driver.performRandomWalk();
      }

      driver.tick();
      //ROS_INFO_STREAM("Current state: " << driver.getStateDescription());
    }
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
