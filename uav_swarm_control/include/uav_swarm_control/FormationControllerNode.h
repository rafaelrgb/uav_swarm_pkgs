/**
 *  This header file defines the FormationControllerNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 26/04/2017
 *  Modified on: 26/04/2017
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _FORMATION_CONTROLLER_NODE_H_
#define _FORMATION_CONTROLLER_NODE_H_

#include <string>
#include "Node.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <uav_swarm_msgs/OdometryWithUavId.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#define PI 3.14159265

namespace uav_swarm_control
{

class FormationControllerNode : public Node
{
public:
  FormationControllerNode(ros::NodeHandle *nh);
  virtual ~FormationControllerNode();

private:
  virtual void controlLoop();

  // UAV params
  bool simulation_;
  int id_;
  double max_vel_;
  double vision_distance_;
  double origin_lat_;
  double origin_lon_;
  std::string fix_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string fixed_frame_;
  std::string tf_frame_;

  // Differences from current position to origin
  double dx_;
  double dy_;
  double dz_;

  // Enable control
  bool enableControl_;
  bool startingPositionCalculated_;
  bool leader_follower_;

  // Rules weights
  double r1_;
  double r2_;
  double r3_;
  double r4_;

  // Position of elements in the system
  nav_msgs::Odometry odom_;
  tf::Vector3 migrationPoint_;
  geometry_msgs::Pose formation_;
  geometry_msgs::Pose leader_formation_;
  std::vector<uav_swarm_msgs::OdometryWithUavId> neighbors_;

  // ROS objects
  ros::Subscriber migration_point_sub_;  // Subscriber to goal
  ros::Subscriber formation_points_sub_; // Subscriber to the formation points
  ros::Subscriber global_position_sub_;  // Subscriber to global position
  ros::Subscriber odom_sub_;             // Subscriber to odometry
  ros::Subscriber enable_control_sub_;   // Subscriber to enable or disable control
  ros::Subscriber uavs_odom_sub_;        // Subscriber to all UAVs odometry
  ros::Publisher uav_odom_pub_;          // UAV odom publisher
  ros::Publisher cmd_vel_pub_;           // Velocity publisher
  tf::TransformBroadcaster pose_br_;
  tf::TransformListener listener_;

  ros::Publisher v1_pub_;
  ros::Publisher v2_pub_;
  ros::Publisher v3_pub_;
  ros::Publisher v4_pub_;
  ros::Publisher vRes_pub_;

  // Member functions
  void migrationPointCb( const geometry_msgs::PointConstPtr &msg );
  void formationPointsCb( const geometry_msgs::PoseArrayConstPtr &msg );
  void globalPositionCb( const sensor_msgs::NavSatFix &msg );
  void odomCb( const nav_msgs::OdometryConstPtr &msg );
  void enableControlCb( const std_msgs::BoolConstPtr &msg );
  void uavsOdomCb( const uav_swarm_msgs::OdometryWithUavIdConstPtr &msg );
  void publishUavOdom ();
  void publishVelocity( double velX, double velY, double velZ );
  void publishVectors( const tf::Vector3 &v1, const tf::Vector3 &v2, const tf::Vector3 &v3,
                       const tf::Vector3 &v4, const tf::Vector3 &vRes );

  // Rules
  void rule1( tf::Vector3& v );
  void rule2( tf::Vector3& v );
  void rule3( tf::Vector3& v );
  void rule4( tf::Vector3& v );


  // Utility functions
  double haversines( double lat1, double lon1, double lat2, double lon2 );

};

}

#endif // _FORMATION_CONTROLLER_NODE_H_
