#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <map>
#include <vector>


tf::Pose tf_pose;

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  tf::poseMsgToTF(pose.pose.pose, tf_pose);
}

void broadcastTf(const ros::TimerEvent&)
{
  static tf::TransformBroadcaster br;

  br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "map", "amcl_pose"));
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "amcl_pose_tf_broadcaster");
  ros::NodeHandle node;

  // Start ROS
  ros::start();

  // Subscribe all aruco markers
  ros::Subscriber sub = node.subscribe("/amcl_pose", 10, &amclCallback);

  // Send transformations every 100 ms
  ros::Timer timer = node.createTimer(ros::Duration(0.5), &broadcastTf, false);

  // Loop
  ros::spin();

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
