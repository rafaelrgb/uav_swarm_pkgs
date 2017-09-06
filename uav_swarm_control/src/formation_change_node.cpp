#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "formation_change_node");
  ros::NodeHandle nh;

  ros::Publisher formation_pub = nh.advertise<geometry_msgs::PoseArray>("/formation_points", 1000);
  ros::Publisher enable_control_pub = nh.advertise<std_msgs::Bool>("/enable_control", 1000);

  // Wait 10 seconds for the simulation to start
  sleep(10);

  // Publish the formation type, enable control
  geometry_msgs::PoseArray formation_msg;
  geometry_msgs::Pose pose;

  // Line
  formation_msg.poses.clear();
  pose.position.x = 5.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    formation_msg.poses.push_back(pose);
    pose.position.x = 0.0;
    pose.position.y = 0.0;
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

    formation_pub.publish(formation_msg);
    std_msgs::Bool enable_msg;
    enable_msg.data = true;

    enable_control_pub.publish(enable_msg);


  // Column
    sleep(10);
    formation_msg.poses.clear();
    pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 0.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);
      pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 5.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);
      pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 10.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);

    formation_pub.publish(formation_msg);


  // Triangle
    sleep(10);
    formation_msg.poses.clear();
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

    formation_pub.publish(formation_msg);


  // Line
    sleep(10);
    formation_msg.poses.clear();
    pose.position.x = 5.0;
      pose.position.y = 0.0;
      pose.position.z = 0.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);
      pose.position.x = 0.0;
      pose.position.y = 0.0;
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

    formation_pub.publish(formation_msg);



  // Triangle
    sleep(10);
    formation_msg.poses.clear();
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

    formation_pub.publish(formation_msg);



  // Column
    sleep(10);
    formation_msg.poses.clear();
    pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 0.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);
      pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 5.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);
      pose.position.x = 0.0;
      pose.position.y = 0.0;
      pose.position.z = 10.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);

    formation_pub.publish(formation_msg);



  // Line
    sleep(10);
    formation_msg.poses.clear();
    pose.position.x = 5.0;
      pose.position.y = 0.0;
      pose.position.z = 0.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      formation_msg.poses.push_back(pose);
      pose.position.x = 0.0;
      pose.position.y = 0.0;
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

    formation_pub.publish(formation_msg);




}
