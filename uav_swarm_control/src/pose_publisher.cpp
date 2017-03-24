#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <uav_swarm_msgs/OdometryWithUavId.h>


ros::Subscriber sub;
ros::Publisher pose1Pub;
ros::Publisher pose2Pub;
ros::Publisher pose3Pub;

void odomCb( const uav_swarm_msgs::OdometryWithUavIdPtr& msg )
{
  geometry_msgs::Pose pose;

  pose.position.x = msg->odom.pose.pose.position.x;
  pose.position.y = msg->odom.pose.pose.position.y;
  pose.position.z = msg->odom.pose.pose.position.z;
  pose.orientation.x = msg->odom.pose.pose.orientation.x;
  pose.orientation.y = msg->odom.pose.pose.orientation.y;
  pose.orientation.z = msg->odom.pose.pose.orientation.z;
  pose.orientation.w = msg->odom.pose.pose.orientation.w;

  if ( msg->id == 0 )
  {
    pose1Pub.publish(pose);
  }
  else if ( msg->id == 1 )
  {
    pose2Pub.publish(pose);
  }
  else
  {
    pose3Pub.publish(pose);
  }
}


int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "pose_publisher");

  ros::NodeHandle nh;

  sub = nh.subscribe("/uavs_odom", 1, odomCb);
  pose1Pub = nh.advertise<geometry_msgs::Pose>("/uav0/pose", 1);
  pose2Pub = nh.advertise<geometry_msgs::Pose>("/uav1/pose", 1);
  pose3Pub = nh.advertise<geometry_msgs::Pose>("/uav2/pose", 1);

  ros::spin();
}
