#include <ros/ros.h>
#include <obstacle_detection/Obstacle.h>
#include <obstacle_detection/ObstacleArray.h>
#include <random>

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
    obstacle_detection::ObstacleArray obstacles;

    // Test code
    for (int i = 0; i < 2; ++i)
    {
      obstacle_detection::Obstacle obs;
      obs.unique_id = std::rand() % 100;

      obs.pose.position.x = std::rand() % 10;
      obs.pose.position.y = std::rand() % 15 + 15;

      obstacles.obstacles.push_back(obs);
    }

    // Populate header
    obstacles.header.stamp = ros::Time::now();

    obstacle_pub.publish(obstacles);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
