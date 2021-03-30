#pragma once

#include <string>

// ROS stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS messages
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

class PositionController
{
    // Members
    // Goal subscribers
    ros::Subscriber goal_rviz_sub_;
    ros::Subscriber goal_rviz2_sub_;
    ros::Subscriber goal_sub_;

    // Publishers
    ros::Publisher twist_pub_;
    ros::Publisher status_pub_;

    // TF listener
    tf::TransformListener tf_listener_;

    // input topics
    std::string input_goal_topic_;
    std::string input_base_frame_;
    float input_goal_distance_threshold_;
    float input_goal_orientation_threshold_;
    
    // output topics
    std::string output_twist_topic_;
    std::string output_status_topic_;

    // Internals
    bool goal_available_;
    float rate_;
    float linear_gain_;
    float heading_gain_;
    float orientation_gain_;

    // 2D goal
    float goal_x_;
    float goal_y_;
    float goal_theta_;
    float initial_distance_to_goal_;
    std::string goal_frame_;

public:
    // Constructor
    PositionController(ros::NodeHandle &nh);

private:
    // Read ROS parameters
    void readParameters(ros::NodeHandle &nh);

    // The callback implements all the actions
    void goalCallback(const geometry_msgs::PoseStamped &in_goal);

    // Main loop
    void run();

    // Utils
    void getRobotPose(float &x, float &y, float &theta);
    double wrapAngle(double x);

};
