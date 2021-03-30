#include <position_controller_cdt/position_controller.h>
#include <Eigen/Dense>
#include <math.h>
#include <limits>

PositionController::PositionController(ros::NodeHandle &nh)
    : goal_available_(false),
      initial_distance_to_goal_(std::numeric_limits<float>::max())
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    goal_rviz_sub_ = nh.subscribe("/goal", 1, &PositionController::goalCallback, this);
    goal_rviz2_sub_ = nh.subscribe("/move_base_simple/goal", 1, &PositionController::goalCallback, this);
    goal_sub_ = nh.subscribe(input_goal_topic_, 1, &PositionController::goalCallback, this);

    // Setup publisher
    twist_pub_  = nh.advertise<geometry_msgs::Twist>(output_twist_topic_, 10);
    status_pub_ = nh.advertise<std_msgs::Float32>(output_status_topic_, 10);

    // Execute main loop
    run();
}

void PositionController::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_goal_topic", input_goal_topic_))
    {
        ROS_ERROR("Could not read parameter `input_goal_topic`.");
        exit(-1);
    }
    if (!nh.getParam("input_base_frame", input_base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_goal_distance_threshold", input_goal_distance_threshold_))
    {
        ROS_ERROR("Could not read parameter `input_goal_distance_threshold`.");
        exit(-1);
    }
    if (!nh.getParam("input_goal_orientation_threshold", input_goal_orientation_threshold_))
    {
        ROS_ERROR("Could not read parameter `input_goal_orientation_threshold`.");
        exit(-1);
    }

    // output topic are optional. They will use default values
    nh.param("output_twist_topic", output_twist_topic_, std::string("/cmd_vel"));
    nh.param("output_status_topic", output_status_topic_, std::string("status"));
    nh.param("control_rate", rate_, 1.f); // 1 second default rate
    nh.param("linear_gain", linear_gain_, 1.f);
    nh.param("heading_gain", heading_gain_, 1.f);
    nh.param("orientation_gain", orientation_gain_, 1.f);
}

void PositionController::goalCallback(const geometry_msgs::PoseStamped &in_goal)
{
    ROS_INFO("Goal received!");
    goal_available_ = true;

    // Extract info form goal
    goal_x_ = in_goal.pose.position.x;
    goal_y_ = in_goal.pose.position.y;
    goal_frame_ = in_goal.header.frame_id;
    initial_distance_to_goal_ = std::numeric_limits<float>::max();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // FIrst we create an Eigen quaternion
    Eigen::Quaterniond q(in_goal.pose.orientation.w,
                         in_goal.pose.orientation.x,
                         in_goal.pose.orientation.y,
                         in_goal.pose.orientation.z);
    // We convert it to an Axis-Angle representation
    // This representatio is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    goal_theta_ = axis_angle.axis().z() * axis_angle.angle();
}

void PositionController::run()
{
    ros::Rate rate(rate_);

    while (ros::ok())
    {

        float vel_x = 0.f;
        float vel_theta = 0.f;
        float distance_error = 0.f;

        if (!goal_available_)
        {
            ROS_DEBUG_THROTTLE(1, "No new goal. This message is throttled (1s)");
        }
        else
        {
            // Get robot pose
            float robot_x, robot_y, robot_theta;
            getRobotPose(robot_x, robot_y, robot_theta);

            // Compute errors from robot to goal
            // Compute error distance to goal (same as before)
            distance_error = std::hypot(goal_x_ - robot_x, goal_y_ - robot_y);
            if(distance_error < input_goal_distance_threshold_)
                distance_error = 0.f;

            // Compute heading error (how much the robot should rotate to point the goal)
            float heading_error = wrapAngle(std::atan2(goal_y_ - robot_y, goal_x_ - robot_x) - robot_theta);

            // Compute orientation error (how much the robot should rotate to match the goal orientation)
            float orientation_error = wrapAngle(goal_theta_ - robot_theta);

            if(initial_distance_to_goal_ == std::numeric_limits<float>::max()){
                initial_distance_to_goal_ = std::max(distance_error - input_goal_distance_threshold_, 0.f);
            }

            ROS_DEBUG_STREAM("\nGoal: (" << goal_x_ << ", " << goal_y_ << ", " << goal_theta_ << "), " << "\n" <<
                            "Robot: (" << robot_x << ", " << robot_y << ", " << robot_theta << "), " << "\n" <<
                            "  distance error:    " << distance_error << "\n" <<
                            "  heading error:     " << heading_error <<  "\n" <<
                            "  orientation error: " << orientation_error <<
                            "  initial_distance_to_goal: " << initial_distance_to_goal_);

            // Check if goal was reached
            if (distance_error < input_goal_distance_threshold_ && fabs(orientation_error) < input_goal_orientation_threshold_)
            {
                goal_available_ = false;
                initial_distance_to_goal_ = std::numeric_limits<float>::max();
                ROS_INFO("Goal reached!");
            }
            else // Compute control command
            {
                // Compute control law (simple proportional (P) controller)
                vel_x = linear_gain_ * distance_error;
                vel_theta = heading_gain_ * heading_error + -orientation_gain_ * orientation_error;
                ROS_INFO_STREAM_THROTTLE(0.2, "Vel: (" << vel_x << ", " << vel_theta << "). This message is throttled (0.2s)");
            }
        }

        // Publish twist command
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = vel_x;
        vel_cmd.linear.y = 0.0;
        vel_cmd.linear.z = 0.0;
        vel_cmd.angular.x = 0.0;
        vel_cmd.angular.y = 0.0;
        vel_cmd.angular.z = vel_theta;
        twist_pub_.publish(vel_cmd);

        // Publish status command
        std_msgs::Float32 status;
        status.data = std::max(1.f - std::max( (distance_error - input_goal_distance_threshold_) / (initial_distance_to_goal_), 0.f), 0.f) * 100.f;
        status_pub_.publish(status);

        ros::spinOnce();
        rate.sleep();
    }
}

void PositionController::getRobotPose(float &x, float &y, float &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(goal_frame_, input_base_frame_,  ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(goal_frame_, input_base_frame_, ros::Time(0), base_to_map_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Extract components from robot pose
    x = base_to_map_transform.getOrigin().getX();
    y = base_to_map_transform.getOrigin().getY();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // First we create an Eigen quaternion
    Eigen::Quaterniond q(base_to_map_transform.getRotation().getW(),
                         base_to_map_transform.getRotation().getX(),
                         base_to_map_transform.getRotation().getY(),
                         base_to_map_transform.getRotation().getZ());
    // We convert it to an Axis-Angle representation
    // This representation is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    theta = axis_angle.axis().z() * axis_angle.angle();
}

double PositionController::wrapAngle(double x)
{
    // This method returns an angle between [-pi, pi]
    x = fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0)
        x += 2.0 * M_PI;
    return x - M_PI;
}