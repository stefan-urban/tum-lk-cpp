#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**
 * Rotates turtlebot to a specific angle, blocks while rotating
 *
 * Usage:
 *  Rotator rot;
 *  rot.startRotation(45.0 / 180 * M_PI); // Rotates turtlebot by 45 degree
 *  nop(); // This will be executed when rotation is finished
 *
 */
class Rotator
{
public:
  const static double ROTATION_SPEED = 1.0;

  /**
   * Resets rotation angle to zero and advertises movement command topic
   */
  Rotator();

  /**
   * Rotates turtlebot a specific angle counterclockwise
   * @param angle: Angle to rotate CCW in rad
   */
  void rotateAngle(float angle);

  /**
   * Rotates turtlebot until a given function handle return true
   * @param function: pointer to a function/method that return true if the
   *                  turtlebot should stop rotating
   */
  void rotateUnilCondition(void* function);

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

  void resetRotation();
  void stopRotation();
  void poseCallback(const nav_msgs::Odometry::ConstPtr& pose);
};
