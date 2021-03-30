/*
 * Based on
 * FiltersDemo.h
 * by Peter Fankhauser (ETH Zurich, ANYbotics)
 */

#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <string>

/*!
 * Applies a chain of grid map filters to a topic and
 * republishes the resulting grid map.
 */
class ElevationMapFilter
{
public:
    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param success signalizes if filter is configured ok or not.
   */
    ElevationMapFilter(ros::NodeHandle &nodeHandle, bool &success);

    /*!
   * Destructor.
   */
    virtual ~ElevationMapFilter();

    /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
    bool readParameters();

    /*!
   * Callback method for the incoming grid map message.
   * @param message the incoming message.
   */
    void callback(const grid_map_msgs::GridMap &message);

private:
    //! ROS nodehandle.
    ros::NodeHandle &nodeHandle_;

    //! Name of the input grid map topic.
    std::string inputTopic_;

    //! Name of the output grid map topic.
    std::string outputTopic_;

    //! Grid map subscriber
    ros::Subscriber subscriber_;

    //! Grid map publisher.
    ros::Publisher publisher_;

    //! Filter chain.
    filters::FilterChain<grid_map::GridMap> filterChain_;

    //! Filter chain parameters name.
    std::string filterChainParametersName_;
};