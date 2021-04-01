#pragma once

#include <string>

// ROS stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// ROS messages
#include <grid_map_msgs/GridMap.h>
#include <cdt_msgs/Graph.h>
#include <cdt_msgs/GraphNode.h>
#include <cdt_msgs/Frontiers.h>
#include <cdt_msgs/ToggleExploration.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>

// frontier handler class
#include <world_explorer_cdt/local_planner.h>
#include <world_explorer_cdt/graph_planner.h>

// Eigen library
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <eigen_conversions/eigen_msg.h>

class WorldExplorer
{
    // Members
    // Service server for starting exploration
    ros::ServiceServer explore_serv_;

    // Subscribers
    ros::Subscriber map_sub_;
    ros::Subscriber frontiers_sub_;
    ros::Subscriber graph_sub_;
    ros::Subscriber position_stat_sub_;

    // Publishers
    ros::Publisher goal_pub_;
    ros::Publisher plan_pub_;

    // TF listener
    tf::TransformListener tf_listener_;

    // input topics
    std::string input_map_topic_;
    std::string input_frontiers_topic_;
    std::string input_graph_topic_;
    std::string input_pos_ctrl_topic_;
    
    // output topics
    std::string output_goal_topic_;
    std::string output_plan_topic_;

    // Internals
    grid_map::GridMap elevation_map_;
    cdt_msgs::Frontiers frontiers_;
    cdt_msgs::Graph exploration_graph_;
    grid_map::GridMap traversability_;

    double rate_;
    double pos_ctrl_status_;
    Eigen::Isometry2d goal_;
    std::string goal_frame_;
    std::string base_frame_;

    double planning_time_ = 0.5;
    double path_execute_limit_ = 100.0;

    bool start_exploring_ = false;
    bool received_frontiers_ = false;
    bool received_trav_map_ = false;
    bool position_controller_ready_ = false;
    bool execute_full_path_ = false;

    // Planers
    LocalPlanner local_planner_;
    GraphPlanner graph_planner_;
    cdt_msgs::Graph graph_;

    // Route
    std::vector<Eigen::Vector2d> route_;
    
public:
    // Constructor
    WorldExplorer(ros::NodeHandle &nh);

private:
    // Read ROS parameters
    void readParameters(ros::NodeHandle &nh);

    // service callback
    bool toggleExploration(cdt_msgs::ToggleExploration::Request &req,
                            cdt_msgs::ToggleExploration::Response & res);

    // The callback implements all the actions
    void elevationMapCallback(const grid_map_msgs::GridMap& in_grid_map);
    void frontiersCallback(const cdt_msgs::Frontiers& in_frontiers);
    void graphCallback(const cdt_msgs::Graph& in_graph);
    void positionCtrlCallback(const std_msgs::Float32& in_status);

    // Main loop
    void run();

    // Plan new targets
    void plan();

    void getRobotPose2D(double &x, double &y, double &theta);
};