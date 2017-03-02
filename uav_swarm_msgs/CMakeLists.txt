cmake_minimum_required(VERSION 2.8.3)
project(uav_swarm_msgs)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
 )

add_message_files(
    DIRECTORY msg
    FILES OdometryWithUavId.msg
)

generate_messages(DEPENDENCIES nav_msgs)
 
catkin_package(
    CATKIN_DEPENDS nav_msgs
)
