#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 * Stopper
 */
class Stopper
{
public:
  // Tunable parameters
  const static double FORWARD_SPEED_MPS = 0.1;
  const static double MIN_SCAN_ANGLE_RAD = -80.0 / 180 * M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +80.0 / 180 * M_PI;
  const static double MIN_PROXIMITY_RANGE_M = 0.8; // Should be smaller than sensor_msgs::LaserScan::range_max

  /**
   *
   */
  Stopper();

  /**
   *
   */
  void startMoving();

private:
  /**
   *
   */
  ros::NodeHandle node;

  /**
   *
   */
  ros::Publisher command_pub;

  /**
   *
   */
  ros::Subscriber laser_sub;

  /**
   *
   */
  bool keep_moving;

  float closest_range;

  /**
   *
   */
  void moveForward();

  /**
   *
   */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

};
