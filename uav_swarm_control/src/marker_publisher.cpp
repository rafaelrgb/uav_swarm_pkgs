#include <stdlib.h>
#include "uav_swarm_control/MarkerPublisher.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "marker_publisher");
  uav_swarm_control::MarkerPublisher node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
