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
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <uav_swarm_msgs/OdometryWithUavId.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define PI 3.14159265

#define MAXVEL   1.0

#define VISION_DISTANCE 3.0

namespace uav_swarm_control
{

class FormationControllerNode : public Node
{
public:
  FormationControllerNode(ros::NodeHandle *nh);
  virtual ~FormationControllerNode();

private:
  virtual void controlLoop();

  // UAV id
  int id_;

  // Differences from current position to origin
  double dx_;
  double dy_;
  bool initialDeltasCalculated_;

  // Enable control
  bool enableControl_;

  // Rules weights
  double r1_;
  double r2_;
  double r3_;
  double r4_;

  // Position of elements in the system
  nav_msgs::Odometry odom_;
  tf::Vector3 migrationPoint_;
  geometry_msgs::Pose formation_;
  std::vector<uav_swarm_msgs::OdometryWithUavId> neighbors_;

  // Flight mode
  std::string mode_;
  bool guided_;
  bool armed_;

  // ROS objects
  ros::Subscriber mavros_state_sub_;     // Subscriber to flight mode
  ros::Subscriber migration_point_sub_;  // Subscriber to goal
  ros::Subscriber formation_points_sub_; // Subscriber to the formation points
  ros::Subscriber global_position_sub_;  // Subscriber to global position
  ros::Subscriber odom_sub_;             // Subscriber to odometry
  ros::Subscriber enable_control_sub_;   // Subscriber to enable or disable control
  ros::Subscriber uavs_odom_sub_;        // Subscriber to all UAVs odometry
  ros::Publisher uav_odom_pub_;          // UAV odom publisher
  ros::Publisher cmd_vel_pub_;           // Velocity publisher
  tf::TransformBroadcaster pose_br_;

  ros::Publisher v1_pub_;
  ros::Publisher v2_pub_;
  ros::Publisher v3_pub_;
  ros::Publisher v4_pub_;
  ros::Publisher vRes_pub_;

  // Member functions
  void mavrosStateCb( const mavros_msgs::StateConstPtr &msg );
  void migrationPointCb( const geometry_msgs::PointConstPtr &msg );
  void formationPointsCb( const geometry_msgs::PoseArrayConstPtr &msg );
  void globalPositionCb( const sensor_msgs::NavSatFix &msg );
  void odomCb( const nav_msgs::OdometryConstPtr &msg );
  void enableControlCb( const std_msgs::BoolConstPtr &msg );
  void uavsOdomCb( const uav_swarm_msgs::OdometryWithUavIdConstPtr &msg );
  void publishUavOdom ();
  void publishVelocity( double velX, double velY );
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
