#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


ros::Subscriber odom1_sub, odom2_sub, odom3_sub, odom4_sub;
ros::Publisher pose1_pub, pose2_pub, pose3_pub, pose4_pub;

void odom1Cb( const nav_msgs::OdometryPtr& msg )
{
  geometry_msgs::Pose pose;

  pose.position.x = msg->pose.pose.position.x;
  pose.position.y = msg->pose.pose.position.y;
  pose.position.z = msg->pose.pose.position.z;
  pose.orientation.x = msg->pose.pose.orientation.x;
  pose.orientation.y = msg->pose.pose.orientation.y;
  pose.orientation.z = msg->pose.pose.orientation.z;
  pose.orientation.w = msg->pose.pose.orientation.w;

  pose1_pub.publish(msg);
}

void odom2Cb( const nav_msgs::OdometryPtr& msg )
{
  geometry_msgs::Pose pose;

  pose.position.x = msg->pose.pose.position.x;
  pose.position.y = msg->pose.pose.position.y;
  pose.position.z = msg->pose.pose.position.z;
  pose.orientation.x = msg->pose.pose.orientation.x;
  pose.orientation.y = msg->pose.pose.orientation.y;
  pose.orientation.z = msg->pose.pose.orientation.z;
  pose.orientation.w = msg->pose.pose.orientation.w;

  pose2_pub.publish(msg);
}

void odom3Cb( const nav_msgs::OdometryPtr& msg )
{
  geometry_msgs::Pose pose;

  pose.position.x = msg->pose.pose.position.x;
  pose.position.y = msg->pose.pose.position.y;
  pose.position.z = msg->pose.pose.position.z;
  pose.orientation.x = msg->pose.pose.orientation.x;
  pose.orientation.y = msg->pose.pose.orientation.y;
  pose.orientation.z = msg->pose.pose.orientation.z;
  pose.orientation.w = msg->pose.pose.orientation.w;

  pose3_pub.publish(msg);
}

void odom4Cb( const nav_msgs::OdometryPtr& msg )
{
  geometry_msgs::Pose pose;

  pose.position.x = msg->pose.pose.position.x;
  pose.position.y = msg->pose.pose.position.y;
  pose.position.z = msg->pose.pose.position.z;
  pose.orientation.x = msg->pose.pose.orientation.x;
  pose.orientation.y = msg->pose.pose.orientation.y;
  pose.orientation.z = msg->pose.pose.orientation.z;
  pose.orientation.w = msg->pose.pose.orientation.w;

  pose4_pub.publish(msg);
}


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "odom_to_pose_converser");
  ros::NodeHandle nh;

  odom1_sub = nh.subscribe("/quadrotor_1/state", 100, odom1Cb);
  odom2_sub = nh.subscribe("/quadrotor_2/state", 100, odom2Cb);
  odom3_sub = nh.subscribe("/quadrotor_3/state", 100, odom3Cb);
  odom4_sub = nh.subscribe("/quadrotor_4/state", 100, odom4Cb);

  pose1_pub = nh.advertise<geometry_msgs::Pose>("/quadrotor_1/pose", 100);
  pose2_pub = nh.advertise<geometry_msgs::Pose>("/quadrotor_2/pose", 100);
  pose3_pub = nh.advertise<geometry_msgs::Pose>("/quadrotor_3/pose", 100);
  pose4_pub = nh.advertise<geometry_msgs::Pose>("/quadrotor_4/pose", 100);

  ros::spin();
}
