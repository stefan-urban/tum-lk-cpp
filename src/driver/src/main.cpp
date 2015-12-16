#include <ros/ros.h>
#include <aruco_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

std::vector<aruco_msgs::Marker> markers;

ros::Publisher cmd_vel_pub;

void markersCallback(const aruco_msgs::MarkerArrayConstPtr& marker_array)
{
//  static tf::TransformListener listener;

  // Copy markers
  markers = marker_array->markers;

  for (aruco_msgs::Marker &marker : markers)
  {
    ROS_INFO_STREAM("found id: " << marker.id);
/*
    tf::StampedTransform transform;

    try
    {
      listener.waitForTransform();
    }
*/
    // Calulate direction
    geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    msg.angular.z = -1.0 * marker.pose.pose.position.x;
    cmd_vel_pub.publish(msg);

    ROS_INFO_STREAM("rotation demand: " << msg.angular.z);
  }
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  static tf::TransformBroadcaster broadcaster;
  tf::Transform transform;

  tf::Quaternion quad;
  tf::quaternionMsgToTF(odom->pose.pose.orientation, quad);

  // Robot is center of universe
  transform.setOrigin(tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0.0));
  transform.setRotation(quad);

  broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv)
{
  // Init
  ros::init(argc, argv, "driver_node");
  ros::NodeHandle node;

  // Start ROS
  ros::start();
  ROS_INFO_STREAM("Startup!");

  // Publisher
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

  // Subscribers
  ros::Subscriber markers_sub = node.subscribe("/aruco_marker_publisher/markers", 1, &markersCallback);
  ros::Subscriber pose_sub = node.subscribe("/odom", 1, &poseCallback);

  // Loop stuff
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
