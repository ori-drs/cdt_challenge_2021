#pragma once

#include <ros/ros.h>
#include <algorithm>
#include <limits>

#include <cdt_msgs/Graph.h>
#include <cdt_msgs/GraphNode.h>

// Eigen library
#include <Eigen/Dense>

class GraphPlanner
{
private:
    cdt_msgs::Graph graph_;
    int graph_size;

public:
    GraphPlanner();
    ~GraphPlanner();

    // Interface to set the graph
    void setGraph(const cdt_msgs::Graph& in_graph);

    // Interface to run the graph planner
    bool planPath(const double& robot_x, const double& robot_y , const double& robot_theta, 
                  Eigen::Vector2d goal_pose,
                  std::vector<Eigen::Vector2d>& route);

private:
    // Internal methods
    void findClosestNodes(const double& robot_x, const double& robot_y , const double& robot_theta, 
                        Eigen::Vector2d goal_pose,
                        std::vector<geometry_msgs::Pose>& nodes);

    // Computes the distance between 2 nodes
    double distance(int id_1, int id_2);

    // Converts the Exploration graph msg into an Eigen matrix
    void generateGraphFromMsg(Eigen::MatrixXd& graph);

    // Dijkstra: minimum distance to start node
    int minimumDist(double dist[], bool Dset[]);  

    // Runs Dijkstra
    void dijkstra(const Eigen::MatrixXd& graph, 
                    int start_id, int goal_id,
                    std::vector<Eigen::Vector2d>& route);

    // Finds the closes node to a certain position
    int getGraphID(double x, double y);
};




