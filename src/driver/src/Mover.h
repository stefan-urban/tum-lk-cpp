#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**
 *
 */
class Mover
{
public:
  /**
   * Advertises movement command topic
   */
  Mover();

  /**
   * Move robot forward until function returns true
   */
  void moveUntilCondition(bool (*function)());

private:
  ros::NodeHandle node;
  ros::Publisher command_pub;
  ros::Subscriber pose_sub;
};
