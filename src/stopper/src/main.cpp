#include <ros/ros.h>
#include <kobuki_msgs/SensorState.h>


bool stop = false;

void sensorsCoreCallback(const kobuki_msgs::SensorStateConstPtr& sensor_state)
{
  if (sensor_state->bumper > 0)
  {
    stop = true;
  }
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "stopper_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Subscribe to sensors
  ros::Subsriber sub = node.subscribe("/mobile_base/sensors/core", 1, &sensorsCoreCallback);

  ros::Rate loop_rate(50.0);

  // Loop
  while (ros::ok())
  {


    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye
  ros::shutdown();

  return 0;
}
