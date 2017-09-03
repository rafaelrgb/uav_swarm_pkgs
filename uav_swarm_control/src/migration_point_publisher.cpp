#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "migration_point_publisher");
  ros::NodeHandle nh;

  ros::Publisher formation_pub = nh.advertise<geometry_msgs::PoseArray>("/formation_points", 1000);
  ros::Publisher migration_point_pub = nh.advertise<geometry_msgs::Point>("/migration_point", 1000);
  ros::Publisher enable_control_pub = nh.advertise<std_msgs::Bool>("/enable_control", 1000);

  // Wait 10 seconds for the simulation to start
  sleep(10);

  // Publish the formation type, enable control
  geometry_msgs::PoseArray formation_msg;
  geometry_msgs::Pose pose;
  // Column
  /*pose.position.x = 7.5;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 2.5;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = -2.5;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = -7.5;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);*/
  // Line
  /*pose.position.x = 0.0;
  pose.position.y = -2.5;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 0.0;
  pose.position.y = 2.5;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 0.0;
  pose.position.y = -7.5;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 0.0;
  pose.position.y = 7.5;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);*/
  // Diamond
  pose.position.x = 5.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 0.0;
  pose.position.y = 5.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 0.0;
  pose.position.y = -5.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = -5.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  // Wedge
  /*pose.position.x = 5.0;
  pose.position.y = 5.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 5.0;
  pose.position.y = -5.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 0.0;
  pose.position.y = 10.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);
  pose.position.x = 0.0;
  pose.position.y = -10.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  formation_msg.poses.push_back(pose);*/


  std_msgs::Bool enable_msg;
  enable_msg.data = true;

  enable_control_pub.publish(enable_msg);
  formation_pub.publish(formation_msg);


  // Wait more 5 seconds for the UAVs to achieve the formation and publish the first migration_point
  sleep(10);

  geometry_msgs::Point migration_point_msg;
  migration_point_msg.x = 30.0;
  migration_point_msg.y = 0.0;
  migration_point_msg.z = 2.0;

  migration_point_pub.publish(migration_point_msg);

  // Wait 20 seconds and publish the second migration point
  sleep(10);
  migration_point_msg.x = 30.0;
  migration_point_msg.y = 30.0;
  migration_point_msg.z = 2.0;

  migration_point_pub.publish(migration_point_msg);

  // Wait 20 seconds and publish the third migration point
  sleep(10);
  migration_point_msg.x = 0.0;
  migration_point_msg.y = 30.0;
  migration_point_msg.z = 2.0;

  migration_point_pub.publish(migration_point_msg);


}
