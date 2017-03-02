#include <stdlib.h>
#include "uav_swarm_control/SwarmControllerNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "swarm_controller_node");
  uav_swarm_control::SwarmControllerNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
