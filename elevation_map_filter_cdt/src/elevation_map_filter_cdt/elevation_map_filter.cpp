/*
 * Based on
 * FiltersDemo.cpp
 * by Peter Fankhauser (ETH Zurich, ANYbotics)
 */

#include <elevation_map_filter_cdt/elevation_map_filter.h>

using namespace grid_map;

ElevationMapFilter::ElevationMapFilter(ros::NodeHandle &nodeHandle, bool &success)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap")
{
  if (!readParameters())
  {
    success = false;
    return;
  }

  subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &ElevationMapFilter::callback, this);
  publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(outputTopic_, 1, true);

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle))
  {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }

  success = true;
}

ElevationMapFilter::~ElevationMapFilter()
{
}

bool ElevationMapFilter::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_))
  {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("output_topic", outputTopic_, std::string("output"));
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  return true;
}

void ElevationMapFilter::callback(const grid_map_msgs::GridMap &message)
{
  // Convert message to map.
  GridMap inputMap;
  GridMapRosConverter::fromMessage(message, inputMap);

  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap))
  {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }

  ROS_INFO("PUBLISH");
  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  publisher_.publish(outputMessage);
}