#include <ros/ros.h>
#include <kobuki_msgs/SensorState.h>

#include "Rotator.h"
#include "Mover.h"


bool stop = false;

void sensorsCoreCallback(const kobuki_msgs::SensorStateConstPtr& sensor_state)
{
  if (sensor_state->bumper > 0)
  {
    stop = true;
  }
}

bool shouldStop()
{
  return stop;
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "driver_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Subscribers
  ros::Subscriber sensors_sub = node.subscribe("/mobile_base/sensors/core", 1, &sensorsCoreCallback);

  // Loop stuff
  ros::Rate loop_rate(50.0);

  Rotator rot;
  Mover mov;

  // Loop
  while (ros::ok())
  {
    mov.moveUntilCondition(&shouldStop);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye
  ros::shutdown();

  return 0;
}
