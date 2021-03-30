#include <cdt_visualizations/explored_space_vis.h>
#include <std_msgs/Int64.h>
#include <Eigen/Dense>
#include <stdexcept>

ExploredSpace::ExploredSpace(ros::NodeHandle &nh)
    : x_last_(0.f),
      y_last_(0.f),
      theta_last_(0.f)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    map_sub_ = nh.subscribe(input_map_topic_, 1, &ExploredSpace::elevationMapCallback, this);

    // Setup publisher
    explored_space_pub_ = nh.advertise<grid_map_msgs::GridMap>(output_explored_space_topic_, 10);
    explored_space_percentage_pub_ = nh.advertise<std_msgs::Int64>(output_explored_space_percentage_topic_, 10);

    // Initialize explored space
    explored_space_.setGeometry(grid_map::Length(explored_space_length_, explored_space_length_), 
                                explored_space_resolution_,
                                grid_map::Position(0.0, 0.0));
    explored_space_.add("elevation", 0.2);
    explored_space_.add("explored_space", 0.0);
    explored_space_.setBasicLayers({"explored_space"});
}

void ExploredSpace::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_map_topic", input_map_topic_))
    {
        ROS_ERROR("Could not read parameter `input_topic`.");
        exit(-1);
    }
    if (!nh.getParam("input_fixed_frame", input_fixed_frame_))
    {
        ROS_ERROR("Could not read parameter `input_fixed_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_base_frame", input_base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }

    // output topic are optional. They will use default values
    nh.param("output_explored_space_topic", output_explored_space_topic_, std::string("/explored_space"));
    nh.param("output_explored_space_percentage_topic", output_explored_space_percentage_topic_, std::string("/explored_space_percentage"));

    nh.param("elevation_threshold", elevation_threshold_, 0.05f);
    nh.param("explored_space_length", explored_space_length_, 50.f);
    nh.param("explored_space_resolution", explored_space_resolution_, 0.5f);
    nh.param("search_angle_resolution", search_angle_resolution_, 10.f);
    nh.param("max_distance_for_explored", max_distance_for_explored_, 5.f);
}

void ExploredSpace::elevationMapCallback(const grid_map_msgs::GridMap &in_grid_map)
{
    // ROS_INFO("New elevation map received!");
    // Convert grid map and store in local variable
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(in_grid_map, grid_map);
    grid_map.convertToDefaultStartIndex();

    // We also need to save the info (header) since it stores the frame and size
    grid_map_msgs::GridMapInfo grid_map_info = in_grid_map.info;

    // Save message time
    ros::Time current_time = in_grid_map.info.header.stamp;

    // Execute the main steps
    // Obtain current pose in fixed frame
    float x, y, theta;
    getRobotPose(x, y, theta);

    // Include a traversability layer in the elevation map
    computeTraversability(grid_map);
    findEdges(x, y, theta, current_time);

    // Update the explored space
    updateExploredSpace(x, y, theta);

    // COmpute the percentage of the world already explored
    computeExploredPercentage();

    // Publish all the data
    publishData(grid_map_info);
}

void ExploredSpace::computeTraversability(const grid_map::GridMap &grid_map)
{
    // Prepare size of grid map
    traversability_.setGeometry(grid_map.getLength(), grid_map.getResolution(), grid_map.getPosition());

    // Copy elevation from input grid map to traversability grid map
    traversability_.add("elevation", grid_map["elevation_inpainted"]);

    // Create a new traversability layer with initial value 0.0
    traversability_.add("traversability", 0.0);

    // Iterate the traversability map to apply a threshold on the height
    for (grid_map::GridMapIterator iterator(traversability_); !iterator.isPastEnd(); ++iterator)
    {
        // We only want to use the valid values
        if (traversability_.isValid(*iterator, "elevation"))
        {
            // Recover elevation at cell
            float elevation = traversability_.at("elevation", *iterator);

            // Check elevation in the cell
            if (elevation > elevation_threshold_)
            {
                // If the elevation is higher than a threshold, cell is untraversable (0.0)
                traversability_.at("traversability", *iterator) = -1.0;
            }
            else
            {
                // If the elevation is smaller than a threshold, cell is traversable (1.0)
                traversability_.at("traversability", *iterator) = 1.0;
            }
        }
    }

    traversability_.setBasicLayers({"traversability", "elevation"});
}

void ExploredSpace::findEdges(const float &x, const float &y, const float &theta, const ros::Time &time)
{
    // Here we use the traversability map to identify main directions the robot should travel to

    // Some constants
    const float half_map_size = 0.5 * traversability_.getLength().x();
    const float step = traversability_.getResolution(); // We use the map resolution as the search step
    ROS_DEBUG_STREAM("half map size: " << half_map_size);
    ROS_DEBUG_STREAM("step: " << step);

    // Clean vector
    new_explored_area_vertices_.clear();

    // We define directions to look for frontiers
    std::vector<grid_map::Position> directions;
    for (float angle = 0.f; angle <= 360.f; angle += search_angle_resolution_)
    {
        grid_map::Position dir(cos(M_PI/180.f * angle),
                               sin(M_PI/180.f * angle));
        directions.push_back(dir);
    }

    // Preallocate query point
    grid_map::Position query_point;
    grid_map::Position robot_position(x, y);

    // Iterate all directions
    for (auto dir : directions)
    {
        // Initially we need a frontier on every direction
        bool needs_frontier = true;

        // Iterate from initial value of step to the map edge
        for (float dis = step; dis < max_distance_for_explored_; dis += step)
        {
            // Create query point
            query_point = dir * dis + robot_position;

            float traversability = 1.f;
            // Check traversability at query point
            try
            {
                traversability = traversability_.atPosition("traversability", query_point);
            }
            catch (const std::out_of_range &oor)
            {
                break;
            }

            // If the traversability is smaller than 1, stop
            // It means there is a wall/obstacle, so the direction was already explored
            if (traversability < 0.f)
            {
                needs_frontier = false;
                break;
            }
        }

        // Add frontier to list to describe the explored area
        new_explored_area_vertices_.push_back(query_point);
        ROS_DEBUG_STREAM("adding vertex: " << query_point << " for explored area");
    }
}

void ExploredSpace::updateExploredSpace(const float &x, const float &y, const float &theta)
{
    // Compute distance to the borders
    float x_offset = (x - (0.5*explored_space_.getLength().x() + explored_space_.getPosition().x()));
    float y_offset = (y - (0.5*explored_space_.getLength().y() + explored_space_.getPosition().y()));
    
    // Move if we reach the borders
    // TODO: Check this in the multi-floor environment
    if(x_offset > 0){
        explored_space_.move(grid_map::Position(x_offset, 0.0));
    }
    if(y_offset > 0){
        explored_space_.move(grid_map::Position(0.0, y_offset));
    }

    // Iterate 
    grid_map::Polygon new_explored_area_polygon(new_explored_area_vertices_);
    for (grid_map::PolygonIterator iterator(explored_space_, new_explored_area_polygon); !iterator.isPastEnd(); ++iterator) {
        explored_space_.at("explored_space", *iterator) = 1.0;
    }
}

void ExploredSpace::computeExploredPercentage()
{
    // Get grid map size
    grid_map::Size size = explored_space_.getSize();

    // Compute ground_truth size
    double ground_truth_size = size.x() * size.y();

    // Compute explored size
    double explored_size = explored_space_["explored_space"].array().sum();

    // Compute percentage explored
    explored_percentage_ = int((explored_size / ground_truth_size) * 100);
}

void ExploredSpace::publishData(const grid_map_msgs::GridMapInfo &in_grid_map_info)
{
    // Publish update explored space
    grid_map_msgs::GridMap explored_space_msg;
    grid_map::GridMapRosConverter::toMessage(explored_space_, explored_space_msg);
    explored_space_msg.info.header = in_grid_map_info.header; // we use the same header from the original msg for consistency
    explored_space_pub_.publish(explored_space_msg);

    // Published percentage of explored space
    std_msgs::Int64 percentage_msg;
    percentage_msg.data = explored_percentage_;
    explored_space_percentage_pub_.publish(percentage_msg);
}

// Utils
void ExploredSpace::getRobotPose(float &x, float &y, float &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(input_fixed_frame_, input_base_frame_, ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(input_fixed_frame_, input_base_frame_, ros::Time(0), base_to_map_transform);
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

double ExploredSpace::wrapAngle(double x)
{
    x = fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0)
        x += 2.0 * M_PI;
    return x - M_PI;
}
