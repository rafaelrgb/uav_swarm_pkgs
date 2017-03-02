cmake_minimum_required(VERSION 2.8.3)
project(uav_swarm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  mavros_msgs
  uav_swarm_msgs
)
 
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp tf std_msgs nav_msgs sensor_msgs geometry_msgs visualization_msgs mavros_msgs
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/Node.h
    include/uav_swarm_control/SwarmControllerNode.h
    include/uav_swarm_control/MarkerPublisher.h
)

set(CPP_FILES
    src/Node.cpp
    src/uav_swarm_control/SwarmControllerNode.cpp
    src/uav_swarm_control/MarkerPublisher.cpp
)

add_executable(swarm_controller_node src/swarm_controller_node.cpp ${CPP_FILES})
add_dependencies(swarm_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_controller_node ${catkin_LIBRARIES})

add_executable(marker_publisher src/marker_publisher.cpp ${CPP_FILES})
add_dependencies(marker_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_publisher ${catkin_LIBRARIES})