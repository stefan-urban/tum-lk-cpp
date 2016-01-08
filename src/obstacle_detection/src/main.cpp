#include <ros/ros.h>
#include <obstacle_detection/Obstacle.h>
#include <obstacle_detection/ObstacleArray.h>

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "obstacle_detection");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Publisher for obstacles
  ros::Publisher obstacle_pub = node.advertise<obstacle_detection::ObstacleArray>("/obstacles", 10);

  ros::Rate loop_rate(0.5);

  while (ros::ok())
  {
    // ... todo

    if (false)
    {
      obstacle_detection::ObstacleArray obstacles;

      obstacle_pub.publish(obstacles);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
