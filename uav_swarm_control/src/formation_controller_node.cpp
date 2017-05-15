#include <stdlib.h>
#include "uav_swarm_control/FormationControllerNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "formation_controller_node");
  uav_swarm_control::FormationControllerNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
