#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**
 * Rotates turtlebot up to 360 degrees
 */
class Rotator
{
public:
  const static double ROTATION_SPEED = 1.0;

  /**
   *
   */
  Rotator();

  /**
   * Rotates turtlebot a specific angle counterclockwise
   * @param float angle: Angle to rotate in rad
   */
  void startRotation(float angle);

  /**
   * Set rotated angle back to zero
   */
  void resetRotation();

  /**
   * Calculates rotation since startRotation()
   */
  float getRotation();

private:
  ros::NodeHandle node;
  ros::Publisher command_pub, angle_pub;
  ros::Subscriber pose_sub;

  float current_angle, initial_angle;
  bool initialized;

  void stopRotation();
  void poseCallback(const nav_msgs::Odometry::ConstPtr& pose);
};
