/**
 *  This source file implements the FormationControllerNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 26/04/2017
 *  Modified on: 26/04/2017
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "uav_swarm_control/FormationControllerNode.h"

namespace uav_swarm_control
{

FormationControllerNode::FormationControllerNode(ros::NodeHandle *nh)
  : Node(nh, 10)
{
    enableControl_ = false;
    migrationPoint_.setValue( 0.0, 0.0, 0.0 );

    ros::param::get("/uav_swarm_control/simulation", simulation_);
    ros::param::get("/uav_swarm_control/fix_topic", fix_topic_);
    ros::param::get("/uav_swarm_control/odom_topic", odom_topic_);
    ros::param::get("/uav_swarm_control/cmd_vel_topic", cmd_vel_topic_);
    ros::param::get("/uav_swarm_control/max_vel", max_vel_);
    ros::param::get("/uav_swarm_control/vision_distance_", vision_distance_);
    ros::param::get("/uav_swarm_control/r1", r1_);
    ros::param::get("/uav_swarm_control/r2", r2_);
    ros::param::get("/uav_swarm_control/r3", r3_);
    ros::param::get("/uav_swarm_control/r4", r4_);

    ros::param::get("uav_id", id_);
    ros::param::get("tf_frame", tf_frame_);
    ros::param::get("x", dx_);
    ros::param::get("y", dy_);
    ros::param::get("z", dz_);

    formation_.position.x = 0.0;
    formation_.position.y = 0.0;
    formation_.position.z = 0.0;
    formation_.orientation.x = 0.0;
    formation_.orientation.y = 0.0;
    formation_.orientation.z = 0.0;
    formation_.orientation.w = 1.0;

    migration_point_sub_ = nh->subscribe("/migration_point", 1, &FormationControllerNode::migrationPointCb, this);
    formation_points_sub_ = nh->subscribe("/formation_points", 1, &FormationControllerNode::formationPointsCb, this);
    global_position_sub_ = nh->subscribe(fix_topic_, 1, &FormationControllerNode::globalPositionCb, this);
    odom_sub_ = nh->subscribe(odom_topic_, 1, &FormationControllerNode::odomCb, this);
    enable_control_sub_ = nh->subscribe("/enable_control", 1, &FormationControllerNode::enableControlCb, this);
    uavs_odom_sub_ = nh->subscribe("/uavs_odom", 10, &FormationControllerNode::uavsOdomCb, this);
    uav_odom_pub_ = nh->advertise<uav_swarm_msgs::OdometryWithUavId>("/uavs_odom", 10);
    //cmd_vel_pub_ = nh->advertise<geometry_msgs::TwistStamped>(cmd_vel_topic_, 10);
    cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

    v1_pub_ = nh->advertise<geometry_msgs::Point>("v1", 10);
    v2_pub_ = nh->advertise<geometry_msgs::Point>("v2", 10);
    v3_pub_ = nh->advertise<geometry_msgs::Point>("v3", 10);
    v4_pub_ = nh->advertise<geometry_msgs::Point>("v4", 10);
    vRes_pub_ = nh->advertise<geometry_msgs::Point>("vRes", 10);
}

FormationControllerNode::~FormationControllerNode()
{
    migration_point_sub_.shutdown();
    formation_points_sub_.shutdown();
    global_position_sub_.shutdown();
    odom_sub_.shutdown();
    enable_control_sub_.shutdown();
    uavs_odom_sub_.shutdown();
    uav_odom_pub_.shutdown();
    cmd_vel_pub_.shutdown();

    v1_pub_.shutdown();
    v2_pub_.shutdown();
    v3_pub_.shutdown();
    v4_pub_.shutdown();
    vRes_pub_.shutdown();
}

void FormationControllerNode::controlLoop()
{
    // Calculate each rule's influence
    tf::Vector3 v1, v2, v3, v4, vRes;
    rule1(v1);
    rule2(v2);
    rule3(v3);
    rule4(v4);

    // Combine the rules
    vRes.setX( r1_ * v1.getX() + r2_ * v2.getX() + r3_ * v3.getX() + r4_ * v4.getX() );
    vRes.setY( r1_ * v1.getY() + r2_ * v2.getY() + r3_ * v3.getY() + r4_ * v4.getY() );
    vRes.setZ( r1_ * v1.getZ() + r2_ * v2.getZ() + r3_ * v3.getZ() + r4_ * v4.getZ() );

    // Limit vRes
    double norm = vRes.length();
    if ( norm >= max_vel_ )
    {
        vRes.normalize();
        vRes *= max_vel_;
    }

    // Publish velocity for diagnostics purpose
    publishVectors( v1, v2, v3, v4, vRes );

    // PUBLISH VELOCITY
    if ( enableControl_ == true )
    {
        // Transform vRes into the quadrotor's base frame
        geometry_msgs::Vector3Stamped stamped_in, stamped_out;

        stamped_in.header.frame_id = "/map";
        stamped_in.vector.x = vRes.getX();
        stamped_in.vector.y = vRes.getY();
        stamped_in.vector.z = vRes.getZ();

        listener_.transformVector(tf_frame_, stamped_in, stamped_out);

        // Retrieve the data from stamped_out and publish it
        tf::Vector3 vResTransformed;
        vResTransformed.setValue(stamped_out.vector.x, stamped_out.vector.y, stamped_out.vector.z);

        publishVelocity( vResTransformed.getX(), vResTransformed.getY(), vResTransformed.getZ() );
    }

    publishUavOdom();


    // TESTE: Imprimir os neighbors para ver se o robô está detectando corretamente
    /*int n = neighbors_.size();
    if ( n > 0 )
    {
        ROS_INFO("UAV %d neighbors:", id_);
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                ROS_INFO("[\n\tid: %d,\n\tx: %f,\n\ty: %f,\n\tz: %f,\n\tvx: %f,\n\tvy: %f,\n\tvz: %f\n]",
                         neighbors_[i].id,
                         neighbors_[i].odom.pose.pose.position.x,
                         neighbors_[i].odom.pose.pose.position.y,
                         neighbors_[i].odom.pose.pose.position.z,
                         neighbors_[i].odom.twist.twist.linear.x,
                         neighbors_[i].odom.twist.twist.linear.y,
                         neighbors_[i].odom.twist.twist.linear.z );
            }
        }
    }*/
}

void FormationControllerNode::publishUavOdom()
{
    uav_swarm_msgs::OdometryWithUavId msg;

    msg.id = id_;
    msg.odom.pose.pose.position.x = odom_.pose.pose.position.x;
    msg.odom.pose.pose.position.y = odom_.pose.pose.position.y;
    msg.odom.pose.pose.position.z = odom_.pose.pose.position.z;
    msg.odom.pose.pose.orientation.x = odom_.pose.pose.orientation.x;
    msg.odom.pose.pose.orientation.y = odom_.pose.pose.orientation.y;
    msg.odom.pose.pose.orientation.z = odom_.pose.pose.orientation.z;
    msg.odom.pose.pose.orientation.w = odom_.pose.pose.orientation.w;
    msg.odom.twist.twist.linear.x = odom_.twist.twist.linear.x;
    msg.odom.twist.twist.linear.y = odom_.twist.twist.linear.y;
    msg.odom.twist.twist.linear.z = odom_.twist.twist.linear.z;

    uav_odom_pub_.publish(msg);
}

void FormationControllerNode::publishVelocity( double velX, double velY, double velZ )
{
    //geometry_msgs::TwistStamped msg;
    geometry_msgs::Twist msg;

    //msg.twist.linear.x = velX;
    //msg.twist.linear.y = velY;
    msg.linear.x = velX;
    msg.linear.y = velY;
    msg.linear.z = velZ;

    cmd_vel_pub_.publish(msg);
}

void FormationControllerNode::publishVectors( const tf::Vector3 &v1, const tf::Vector3 &v2, const tf::Vector3 &v3,
                                          const tf::Vector3 &v4, const tf::Vector3 &vRes )
{
    geometry_msgs::Point v1Msg, v2Msg, v3Msg, v4Msg, vResMsg;
    v1Msg.x = v1.getX();
    v1Msg.y = v1.getY();
    v1Msg.z = v1.getZ();
    v2Msg.x = v2.getX();
    v2Msg.y = v2.getY();
    v2Msg.z = v2.getZ();
    v3Msg.x = v3.getX();
    v3Msg.y = v3.getY();
    v3Msg.z = v3.getZ();
    v4Msg.x = v4.getX();
    v4Msg.y = v4.getY();
    v4Msg.z = v4.getZ();
    vResMsg.x = vRes.getX();
    vResMsg.y = vRes.getY();
    vResMsg.z = vRes.getZ();
    v1_pub_.publish( v1Msg );
    v2_pub_.publish( v2Msg );
    v3_pub_.publish( v3Msg );
    v4_pub_.publish( v4Msg );
    vRes_pub_.publish( vResMsg );
}

void FormationControllerNode::migrationPointCb( const geometry_msgs::PointConstPtr &msg )
{
    migrationPoint_.setX( msg->x );
    migrationPoint_.setX( msg->y );
    migrationPoint_.setZ( msg->z );
}

void FormationControllerNode::formationPointsCb(const geometry_msgs::PoseArrayConstPtr &msg )
{
    if ( msg->poses.size() > id_ )
    {
        formation_.position.x = msg->poses[id_].position.x;
        formation_.position.y = msg->poses[id_].position.y;
        formation_.position.z = msg->poses[id_].position.z;
        formation_.orientation.x = msg->poses[id_].orientation.x;
        formation_.orientation.y = msg->poses[id_].orientation.y;
        formation_.orientation.z = msg->poses[id_].orientation.z;
        formation_.orientation.w = msg->poses[id_].orientation.w;
    }
}

void FormationControllerNode::globalPositionCb( const sensor_msgs::NavSatFix &msg )
{
    // Quando a primeira mensagem de GPS chegar, calcular dx_ e dy_
    /*if ( !initialDeltasCalculated_ )
    {
        double originLat, originLon;
        ros::param::get("/uav_swarm_control/origin_lat", originLat);
        ros::param::get("/uav_swarm_control/origin_lon", originLon);
        double lat = msg.latitude;
        double lon = msg.longitude;
        dx_ = haversines( originLat, originLon, originLat, lon );
        dy_ = haversines( originLat, originLon, lat, originLon );
        if ( (lon - originLon) < 0 ) dx_ *= -1;
        if ( (lat - originLat) < 0 ) dy_ *= -1;
        initialDeltasCalculated_ = true;
    }*/
}

void FormationControllerNode::odomCb( const nav_msgs::OdometryConstPtr &msg )
{
    // Get UAV odometry from the received message
    // If ground truth data from the simulation is being used, ignore the deltas
    if (simulation_)
    {
      odom_.pose.pose.position.x = msg->pose.pose.position.x;
      odom_.pose.pose.position.y = msg->pose.pose.position.y;
      odom_.pose.pose.position.z = msg->pose.pose.position.z;
    }
    else
    {
      odom_.pose.pose.position.x = msg->pose.pose.position.x + dx_;
      odom_.pose.pose.position.y = msg->pose.pose.position.y + dy_;
      odom_.pose.pose.position.z = msg->pose.pose.position.z + dz_;
    }
    odom_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    odom_.twist.twist.linear.z = msg->twist.twist.linear.z;

    // Publish to tf
    pose_br_.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(
                    odom_.pose.pose.orientation.x,
                    odom_.pose.pose.orientation.y,
                    odom_.pose.pose.orientation.z,
                    odom_.pose.pose.orientation.w
                ),
                tf::Vector3(
                    odom_.pose.pose.position.x,
                    odom_.pose.pose.position.y,
                    odom_.pose.pose.position.z
                )
            ), ros::Time::now(), "map", tf_frame_
        )
    );
}

void FormationControllerNode::enableControlCb( const std_msgs::BoolConstPtr &msg )
{
    enableControl_ = msg->data;
}

void FormationControllerNode::uavsOdomCb( const uav_swarm_msgs::OdometryWithUavIdConstPtr &msg )
{
    // Only consider messages that came from neighbors
    if ( msg->id != id_ )
    {
        bool isNew = true;

        // Loop through the neighbors array
        for ( int i = 0; i < neighbors_.size(); i++ )
        {
            // Update existing neighbors...
            if ( msg->id == neighbors_[i].id )
            {
                neighbors_[i].odom.pose.pose.position.x = msg->odom.pose.pose.position.x;
                neighbors_[i].odom.pose.pose.position.y = msg->odom.pose.pose.position.y;
                neighbors_[i].odom.pose.pose.position.z = msg->odom.pose.pose.position.z;
                neighbors_[i].odom.pose.pose.orientation.x = msg->odom.pose.pose.orientation.x;
                neighbors_[i].odom.pose.pose.orientation.y = msg->odom.pose.pose.orientation.y;
                neighbors_[i].odom.pose.pose.orientation.z = msg->odom.pose.pose.orientation.z;
                neighbors_[i].odom.pose.pose.orientation.w = msg->odom.pose.pose.orientation.w;
                neighbors_[i].odom.twist.twist.linear.x = msg->odom.twist.twist.linear.x;
                neighbors_[i].odom.twist.twist.linear.y = msg->odom.twist.twist.linear.y;
                neighbors_[i].odom.twist.twist.linear.z = msg->odom.twist.twist.linear.z;

                isNew = false;
                break;
            }
        }

        // ...or add new neighbors
        if ( isNew == true )
        {
            uav_swarm_msgs::OdometryWithUavId odomWithUavId;
            odomWithUavId.id = msg->id;
            odomWithUavId.odom.pose.pose.position.x = msg->odom.pose.pose.position.x;
            odomWithUavId.odom.pose.pose.position.y = msg->odom.pose.pose.position.y;
            odomWithUavId.odom.pose.pose.position.z = msg->odom.pose.pose.position.z;
            odomWithUavId.odom.pose.pose.orientation.x = msg->odom.pose.pose.orientation.x;
            odomWithUavId.odom.pose.pose.orientation.y = msg->odom.pose.pose.orientation.y;
            odomWithUavId.odom.pose.pose.orientation.z = msg->odom.pose.pose.orientation.z;
            odomWithUavId.odom.pose.pose.orientation.w = msg->odom.pose.pose.orientation.w;
            odomWithUavId.odom.twist.twist.linear.x = msg->odom.twist.twist.linear.x;
            odomWithUavId.odom.twist.twist.linear.y = msg->odom.twist.twist.linear.y;
            odomWithUavId.odom.twist.twist.linear.z = msg->odom.twist.twist.linear.z;
            neighbors_.push_back(odomWithUavId);
        }
    }
}


// REYNOLDS RULES

// Rule 1: Flocking
void FormationControllerNode::rule1( tf::Vector3& v )
{
    v.setValue( 0.0, 0.0, 0.0 );

    int n = neighbors_.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                tf::Vector3 neighborPosition(
                    neighbors_[i].odom.pose.pose.position.x,
                    neighbors_[i].odom.pose.pose.position.y,
                    neighbors_[i].odom.pose.pose.position.z
                );
                v += neighborPosition;
            }
        }

        v /= n;

        tf::Vector3 thisPosition(
            odom_.pose.pose.position.x,
            odom_.pose.pose.position.y,
            odom_.pose.pose.position.z
        );
        v -= thisPosition;
    }
}


// Rule 2: Collision Avoidance
void FormationControllerNode::rule2( tf::Vector3& v )
{
    v.setValue( 0.0, 0.0, 0.0 );

    int n = neighbors_.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                tf::Vector3 thisPosition(
                    odom_.pose.pose.position.x,
                    odom_.pose.pose.position.y,
                    odom_.pose.pose.position.z
                );
                tf::Vector3 neighborPosition(
                    neighbors_[i].odom.pose.pose.position.x,
                    neighbors_[i].odom.pose.pose.position.y,
                    neighbors_[i].odom.pose.pose.position.z
                );

                double d = thisPosition.distance( neighborPosition );

                if ( d < vision_distance_ )
                {
                    if ( d < 0.01 ) d = 0.1;
                    double dif = vision_distance_ - d;

                    thisPosition -= neighborPosition;
                    thisPosition /= d;
                    thisPosition *= dif;

                    v += thisPosition;
                }
            }
        }
    }
}

// Rule 3: Velocity Matching
void FormationControllerNode::rule3( tf::Vector3& v )
{
    v.setValue( 0.0, 0.0, 0.0 );

    int n = neighbors_.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                tf::Vector3 neighborVelocity(
                    neighbors_[i].odom.twist.twist.linear.x,
                    neighbors_[i].odom.twist.twist.linear.y,
                    neighbors_[i].odom.twist.twist.linear.z
                );
                v += neighborVelocity;
            }
        }

        v /= n;

        /*
        tf::Vector3 thisVelocity(
            odom_.twist.twist.linear.x,
            odom_.twist.twist.linear.y,
            odom_.twist.twist.linear.z
        );
        v -= thisVelocity;*/
    }
}


// Rule 4: Migration
void FormationControllerNode::rule4( tf::Vector3& v )
{
    v.setValue( 0.0, 0.0, 0.0 );

    int n = neighbors_.size();

    if ( n > 0 )
    {
        if ( id_ != 0 )
        {
            // Move according to leader
            tf::Vector3 leader( 0.0, 0.0, 0.0 );
            for ( int i(0); i < n; i++ )
            {
                if ( neighbors_[i].id == 0 )
                {
                    leader.setX( neighbors_[i].odom.pose.pose.position.x );
                    leader.setY( neighbors_[i].odom.pose.pose.position.y );
                    leader.setZ( neighbors_[i].odom.pose.pose.position.z );
                }
            }

            v.setX( formation_.position.x + leader.getX() - odom_.pose.pose.position.x );
            v.setY( formation_.position.y + leader.getY() - odom_.pose.pose.position.y );
            v.setZ( formation_.position.z + leader.getZ() - odom_.pose.pose.position.z );
        }
        else
        {
            // I am the leader
            v.setX( formation_.position.x + migrationPoint_.getX() - odom_.pose.pose.position.x );
            v.setY( formation_.position.y + migrationPoint_.getY() - odom_.pose.pose.position.y );
            v.setZ( formation_.position.z + migrationPoint_.getZ() - odom_.pose.pose.position.z );
        }
    }
}


// Returns the distance in meters between 2 points given their GPS coordinates
double FormationControllerNode::haversines( double lat1, double lon1, double lat2, double lon2 )
{
    double a, c, d, dLat, dLon;
    int r = 6371000; // raio médio da Terra em metros
    // converter os ângulos para radianos:
    double degToRad = PI / 180.0;
    lat1 *= degToRad;
    lat2 *= degToRad;
    lon1 *= degToRad;
    lon2 *= degToRad;
    dLat = lat2 - lat1;
    dLon = lon2 - lon1;

    // fórmula de haversines:
    a = sin( dLat / 2 ) * sin( dLat / 2 ) +
               cos( lat1 ) * cos( lat2 ) *
               sin( dLon / 2 ) * sin( dLon / 2 );
    c = 2 * atan2( sqrt( a ), sqrt( 1 - a ) );
    d = r * c;

    return d;
}

}
