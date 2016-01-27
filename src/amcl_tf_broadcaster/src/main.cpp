#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>

void amclposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  static tf::TransformBroadcaster br;

  tf::Pose tf_pose;
  tf::poseMsgToTF(pose->pose.pose, tf_pose);

  br.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(), "map", "amcl"));
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "amcl_tf_broadcaster_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();

  // Subscribe all aruco markers
  ros::Subscriber sub = node.subscribe("/amcl_pose", 10, &amclposeCallback);

  // Loop
  ros::spin();

  // Good bye turtlebot
  ros::shutdown();

  return 0;
}
