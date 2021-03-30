#pragma once

#include <string>

// ROS stuff
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

// ROS messages
#include <sensor_msgs/Image.h>
#include <cdt_msgs/ObjectList.h>
#include <cdt_msgs/Object.h>

// Eigen library
#include <Eigen/Dense>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#define MIN_BBOX_WIDTH 50
#define MIN_BBOX_HEIGHT 50
#define MIN_OBJECT_DEPTH 1

// Colours
enum Colour { RED=0, YELLOW=0, GREEN=1,  BLUE=2 };

class ObjectDetector
{
    // Members
    // Image subscriber
    image_transport::Subscriber image_sub_;

    // Detected objects publisher
    ros::Publisher objects_pub_;

    // TF listener
    tf::TransformListener tf_listener_;

    // List of detected objects
    cdt_msgs::ObjectList detected_objects_;

    // Input topics
    std::string input_image_topic_;
    std::string base_frame_;
    std::string fixed_frame_;

    // Output topics
    std::string output_objects_topic_;

    // Internals
    double camera_extrinsic_x_; // Position from base_link to camera coordinates (in meters)
    double camera_extrinsic_y_; // Position from base_link to camera coordinates (in meters)
    double camera_extrinsic_z_; // Position from base_link to camera coordinates (in meters)

    double camera_fx_; // Intrinsic calibration: Focal length (in pixels)
    double camera_fy_; // Intrinsic calibration: Focal length (in pixels)
    double camera_cx_; // Intrinsic calibration: Camera center (in pixels)
    double camera_cy_; // Intrinsic calibration: Camera center (in pixels)

    // Objects' heights
    double barrel_real_height_, barrow_real_height_, computer_real_height_, dog_real_height_;
    
public:
    // Constructor
    ObjectDetector(ros::NodeHandle &nh);

private:
    // Read ROS parameters
    void readParameters(ros::NodeHandle &nh);

    // The callback implements all the actions
    void imageCallback(const sensor_msgs::ImageConstPtr &in_msg);

    // Converts message to opencv image
    void convertMessageToImage(const sensor_msgs::ImageConstPtr &in_msg, cv::Mat& out_image, ros::Time& out_timestamp);

    // Detects the specified colour within the input image (BGR)
    cv::Mat applyColourFilter(const cv::Mat &in_image_bgr, const Colour &colour);

    // Detects the specified colour within the input image (BGR)
    cv::Mat applyBoundingBox(const cv::Mat1b &in_mask, double &x, double &y, double &width, double &height);

    // Implements the procedures to recognize objects
    bool recognizeDog(const cv::Mat &in_image, const ros::Time &in_timestamp, 
                      const double& robot_x, const double& robot_y, const double& robot_theta,
                      cdt_msgs::Object &out_new_object);

    
    // Utils
    void getRobotPose(double &x, double &y, double &theta);
    bool wasObjectDetected(std::string object_name);
};