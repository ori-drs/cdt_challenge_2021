/*
 * Based on
 * filters_demo_node.cpp
 * by Peter Fankhauser (ETH Zurich, ANYbotics)
 */

#include <elevation_map_filter_cdt/elevation_map_filter.h>

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevation_map_filter");
  ros::NodeHandle nodeHandle("~");
  bool success;
  ElevationMapFilter filter(nodeHandle, success);
  if (success)
    ros::spin();
  return 0;
}
