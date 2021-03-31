#pragma once

#include <string>
#include <Eigen/Dense>
#include <stdexcept>

// ROS stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS messages
#include <grid_map_msgs/GridMap.h>
#include <cdt_msgs/Graph.h>
#include <cdt_msgs/GraphNode.h>
#include <cdt_msgs/Frontiers.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <opencv2/core.hpp>

class WorldModelling
{
    // Members
    // Elevation map subscriber
    ros::Subscriber map_sub_;

    // Publishers
    ros::Publisher graph_pub_;
    ros::Publisher traversability_pub_;
    ros::Publisher frontiers_pub_;
    ros::Publisher explored_space_pub_;

    // TF listener
    tf::TransformListener tf_listener_;

    // input topics/parameters
    std::string input_map_topic_;
    std::string input_fixed_frame_;
    std::string input_base_frame_;
    float neighbor_distance_;
    int neighbor_path_distance_;
    float node_creation_distance_;
    float elevation_threshold_;
    float max_distance_to_search_frontiers_;
    float distance_to_delete_frontier_;
    float frontiers_search_angle_resolution_;

    // output topics
    std::string output_graph_topic_;
    std::string output_frontiers_topic_;
    std::string output_traversability_topic_;

    // Internals
    cdt_msgs::Graph exploration_graph_;
    grid_map::GridMap traversability_;
    cdt_msgs::Frontiers current_frontiers_;
    cdt_msgs::Frontiers frontiers_;

    // Last added pose
    float x_last_;
    float y_last_;
    float theta_last_;
    unsigned int num_nodes_;
    bool first_node_;
    bool first_frontier_;

public:
    // Constructor
    WorldModelling(ros::NodeHandle &nh);

private:
    // Read ROS parameters
    void readParameters(ros::NodeHandle &nh);

    // The callback implements all the actions
    void elevationMapCallback(const grid_map_msgs::GridMap&  in_grid_map);

    // Main operations
    // Updates the exploration graph new nodes
    bool updateGraph(const float &x, const float &y, const float &theta);

    // Computes the traversability map
    void computeTraversability(const grid_map::GridMap& grid_map);

    // Computes frontiers from current grid map
    void findCurrentFrontiers(const float &x, const float &y, const float &theta, const ros::Time& time);

    // Updates the history of frontiers to be used by the explorer
    void updateFrontiers(const float &x, const float &y, const float &theta);

    // Publishes all the data that was computed
    void publishData(const grid_map_msgs::GridMapInfo& in_grid_map_info);

    // Utils
    void getRobotPose(float &x, float &y, float &theta);
    float distanceBetweenNodes(cdt_msgs::GraphNode n1, cdt_msgs::GraphNode n2);
};