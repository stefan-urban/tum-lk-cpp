#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pathfinder/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "TargetDetermination.h"
#include "Planner.h"

const std::string global_frame_id = "/map";
const std::string robot_frame_id = "/base_link";

geometry_msgs::PoseStamped current_position;

void positionUpdate(const ros::TimerEvent&)
{
  static unsigned int error_counter = 0;
  static unsigned int seq = 0;

  // Retrieve current position on map
  tf::StampedTransform transform;
  static tf::TransformListener listener;

  try
  {
    listener.lookupTransform(robot_frame_id, global_frame_id,  ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM("Current position on map could not be determined. (" << error_counter++ << ")");
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set timestamp
  current_position.header.seq = seq++;
  current_position.header.stamp = ros::Time::now();
  current_position.header.frame_id = global_frame_id;

  // Determine position
	geometry_msgs::Vector3 pos;
  tf::vector3TFToMsg(transform.getOrigin(), pos);

  current_position.pose.position.x = pos.x;
  current_position.pose.position.y = pos.y;
  current_position.pose.position.z = pos.z;

  // Determine orientation
  tf::quaternionTFToMsg(transform.getRotation(), current_position.pose.orientation);
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "pathfinder");
  ros::NodeHandle node;

  // Start ROS
  ros::start();

  // Periodically update own position
  ros::Timer timer = node.createTimer(ros::Duration(0.1), &positionUpdate);

  // Wait for first position update
  while (ros::ok() && current_position.header.stamp.sec == 0)
  {
    ros::Duration(0.2).sleep();
  }

  // Advertise paths topic
  ros::Publisher path_pub = node.advertise<pathfinder::Path>("/paths", 10);

  // Setup target determination
  TargetDetermination td;
  std::map<unsigned int, geometry_msgs::PoseStamped> goals;

  // Setup path planner
  Planner p;

  // Loop
  ros::Rate loop_rate(1.0);

  while (ros::ok())
  {
    // Get position of all aruco markers
    goals = td.getGoals();

    // Calculate path for each single goal
    for (const auto &goal : goals)
    {
      // Calc path
      nav_msgs::Path path = p.makePlan(current_position, goal.second);

      // Debug
      p.debug_broadcast_tf(goal.first, path);

      // Publish path
      pathfinder::Path msg;

      msg.destination_id = goal.first;
      msg.path = path;

      path_pub.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
