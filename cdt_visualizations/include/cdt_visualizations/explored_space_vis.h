#pragma once

#include <string>

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

class ExploredSpace
{
    // Members
    // Elevation map subscriber
    ros::Subscriber map_sub_;

    // Publishers
    ros::Publisher explored_space_pub_;
    ros::Publisher explored_space_percentage_pub_;

    // TF listener
    tf::TransformListener tf_listener_;

    // input topics/parameters
    std::string input_map_topic_;
    std::string input_fixed_frame_;
    std::string input_base_frame_;
    float elevation_threshold_;
    float explored_space_length_;
    float explored_space_resolution_;
    float search_angle_resolution_;
    float max_distance_for_explored_;

    // output topics
    std::string output_explored_space_topic_;
    std::string output_explored_space_percentage_topic_;

    // Explored space
    grid_map::GridMap explored_space_;
    grid_map::GridMap traversability_;
    std::vector<grid_map::Position> new_explored_area_vertices_;

    // Last added pose
    float x_last_;
    float y_last_;
    float theta_last_;
    int explored_percentage_;

public:
    // Constructor
    ExploredSpace(ros::NodeHandle &nh);

private:
    // Read ROS parameters
    void readParameters(ros::NodeHandle &nh);

    // The callback implements all the actions
    void elevationMapCallback(const grid_map_msgs::GridMap&  in_grid_map);

    // Main operations
    // Computes the traversability map
    void computeTraversability(const grid_map::GridMap& grid_map);

    // Computes frontiers from current grid map
    void findEdges(const float &x, const float &y, const float &theta, const ros::Time& time);

    // Updates the area already explored
    void updateExploredSpace(const float &x, const float &y, const float &theta);
    void computeExploredPercentage();

    // Publishes all the data that was computed
    void publishData(const grid_map_msgs::GridMapInfo& in_grid_map_info);

    // Utils
    void getRobotPose(float &x, float &y, float &theta);
    double wrapAngle(double x);
};