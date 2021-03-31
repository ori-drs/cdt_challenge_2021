#include <world_modelling_cdt/world_modelling.h>

WorldModelling::WorldModelling(ros::NodeHandle &nh)
    : x_last_(0.f),
      y_last_(0.f),
      theta_last_(0.f),
      num_nodes_(0),
      first_node_(true),
      first_frontier_(true)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    map_sub_ = nh.subscribe(input_map_topic_, 1, &WorldModelling::elevationMapCallback, this);

    // Setup publisher
    graph_pub_ = nh.advertise<cdt_msgs::Graph>(output_graph_topic_, 10);
    frontiers_pub_ = nh.advertise<cdt_msgs::Frontiers>(output_frontiers_topic_, 10);
    traversability_pub_ = nh.advertise<grid_map_msgs::GridMap>(output_traversability_topic_, 10);
}

void WorldModelling::readParameters(ros::NodeHandle &nh)
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
    nh.param("output_graph_topic", output_graph_topic_, std::string("/exploration_graph"));
    nh.param("output_traversability_topic", output_traversability_topic_, std::string("/traversability"));
    nh.param("output_frontiers_topic", output_frontiers_topic_, std::string("/frontiers"));
    nh.param("node_creation_distance", node_creation_distance_, 1.f);
    nh.param("neighbor_distance", neighbor_distance_, 2.f);
    nh.param("elevation_threshold", elevation_threshold_, 0.1f);

    nh.param("max_distance_to_search_frontiers", max_distance_to_search_frontiers_, 3.f);
    nh.param("distance_to_delete_frontier", distance_to_delete_frontier_, 2.5f);
    nh.param("frontiers_search_angle_resolution", frontiers_search_angle_resolution_, 0.5f);
}

void WorldModelling::elevationMapCallback(const grid_map_msgs::GridMap &in_grid_map)
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

    // Update and publish the pose graph with current pose
    bool new_node = updateGraph(x, y, theta);

    // Include a traversability layer in the elevation map
    computeTraversability(grid_map);

    // We only look for frontiers if we are adding a new node in the graph
    if (new_node)
    {
        // Compute, update and publish the frontiers
        findCurrentFrontiers(x, y, theta, current_time);
        updateFrontiers(x, y, theta);
    }

    // Publish all the data
    publishData(grid_map_info);
}

bool WorldModelling::updateGraph(const float &x, const float &y, const float &theta)
{
    // TODO: you need to update the exploration_graph_ using the current pose

    // You may need to change this flag with some conditions
    bool create_new_node = true;

    // if the condition is satisfied, you should create a new node and add it to the graph
    if(first_node_)
    {
        // Here we briefly show how to fill the data
        cdt_msgs::GraphNode new_node;
        new_node.pose.position.x = x;
        new_node.pose.position.y = y;
        new_node.id.data = num_nodes_; // The id is simply the number of the node
        
        // Adding neighbors
        std_msgs::Int32 neighbor_id;
        neighbor_id.data = 0;
        new_node.neighbors_id.push_back(neighbor_id);  // here we fill the neighbors of the new_node

        // Finally add the new node to the graph (since all the properties are filled)
        exploration_graph_.nodes.push_back(new_node);

        num_nodes_++; // Increase the number of added nodes

        first_node_ = false; // This is to avoid creating more than one nodes. This is just for the example, you may need to remove this

        return true;
    }
    else
    {
        // We didn't create a new node
        return false;
    }
}

void WorldModelling::computeTraversability(const grid_map::GridMap &grid_map)
{
    // Prepare size of grid map
    traversability_.setGeometry(grid_map.getLength(), grid_map.getResolution(), grid_map.getPosition());

    // Copy elevation from input grid map to traversability grid map
    traversability_.add("elevation", grid_map["elevation_inpainted"]);
    // Copy slope from input grid map to traversability grid map
    traversability_.add("slope", grid_map["slope_inpainted"]);

    // Create a new traversability layer with initial value 0.0
    traversability_.add("traversability", 0.0);

    // Iterate the traversability map to apply a threshold on the height
    for (grid_map::GridMapIterator iterator(traversability_); !iterator.isPastEnd(); ++iterator)
    {
        // We only want to use the valid values
        if (traversability_.isValid(*iterator, "elevation"))
        {

            // TODO Fill the traversability at each position using some criterion based on the other layers
            // How can we figure out if an area is traversable or not?
            // YOu should fill with a 1.0 if it's traversable, and -1.0 in the other case
            traversability_.at("traversability", *iterator) = -1.0;
        }
    }

    traversability_.setBasicLayers({"traversability", "elevation"});
}

void WorldModelling::findCurrentFrontiers(const float &x, const float &y, const float &theta, const ros::Time &time)
{
    // TODO: Here you need to create "frontiers" that denote the edges of the known space
    // They're used to guide robot to new places

    // If the direction needs a frontier, create one and store in current frontiers
    if (first_frontier_)
    {
        // We create the frontier as a stamped point

        // In this example we set a frontier 5 meters ahead of the robot
        // The frontiers are expresed in the fixed frame
        geometry_msgs::PointStamped frontier;
        frontier.header.stamp = time;                  // We store the time the frontier was created
        frontier.header.frame_id = input_fixed_frame_; // And the frame it's referenced to
        frontier.point.x = x + 5.0;                    // And the position, of course
        frontier.point.y = y;

        // Finally we store it in the current frontiers' list
        current_frontiers_.frontiers.push_back(frontier);

        first_frontier_ = false; // This is to avoid creating more than one frontiers. This is just for the example, you may need to remove this
    }

    ROS_DEBUG_STREAM("[WorldModelling] Found " << current_frontiers_.frontiers.size() << " new frontiers");
}

void WorldModelling::updateFrontiers(const float &x, const float &y, const float &theta)
{
    // We need to combine the accumulated frontiers with the current ones

    // We will iterate all the accumulated frontiers and check some conditions:
    // 1. If they are closer to the robot (hence, already explored)
    // 2. If there is any wall in between
    // The frontiers that pass the test will be added to the current frontiers

    // Some constants
    float half_map_size = 0.5 * traversability_.getLength().x();
    const float step = traversability_.getResolution(); // We use the map resolution as the search step

    // Filtered frontiers
    cdt_msgs::Frontiers filtered_frontiers;

    // Preallocate query point
    grid_map::Position query_point;

    // Iterate
    for (auto frontier : frontiers_.frontiers)
    {
        const float &frontier_x = frontier.point.x;
        const float &frontier_y = frontier.point.y;

        // Compute distance to frontier
        float distance_to_frontier = std::hypot(frontier_x - x, frontier_y - y);

        // If it's close enough, skip
        if (distance_to_frontier < distance_to_delete_frontier_)
        {
            continue;
        }

        // If the previous test are passed, add frontier to filtered list
        current_frontiers_.frontiers.push_back(frontier);
    }

    // Finally, we update the frontiers using the current ones
    frontiers_ = current_frontiers_;
}

void WorldModelling::publishData(const grid_map_msgs::GridMapInfo &in_grid_map_info)
{
    // Publish exploration graph
    graph_pub_.publish(exploration_graph_);

    // Publish traversability map
    grid_map_msgs::GridMap traversability_msg;
    grid_map::GridMapRosConverter::toMessage(traversability_, traversability_msg);
    traversability_msg.info.header = in_grid_map_info.header; // we use the same header from the original msg for consistency
    traversability_pub_.publish(traversability_msg);

    // Publish updated list of frontiers
    frontiers_pub_.publish(frontiers_);
}

// Utils
void WorldModelling::getRobotPose(float &x, float &y, float &theta)
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