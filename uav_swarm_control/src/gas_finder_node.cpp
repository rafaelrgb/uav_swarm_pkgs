#include <stdlib.h>
#include "uav_swarm_control/GasFinderNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "gas_finder_node");
  uav_swarm_control::GasFinderNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
