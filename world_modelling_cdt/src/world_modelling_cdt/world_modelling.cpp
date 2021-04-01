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
    nh.param("neighbor_distance", neighbor_distance_, 1.f);
    nh.param("elevation_threshold", elevation_threshold_, 0.1f);
    nh.param("neighbor_path_distance", neighbor_path_distance_, 5);

    nh.param("max_distance_to_search_frontiers", max_distance_to_search_frontiers_, 3.f);
    nh.param("distance_to_delete_frontier", distance_to_delete_frontier_, 2.5f);
    nh.param("frontier_search_angle_resolution", frontier_search_angle_resolution_, 30.f);
    nh.param("frontier_search_angle", frontier_search_angle_, 180.f);
    nh.param("frontier_min_separation", frontier_min_separation_, 2.f);

    nh.param("trav_check_distance", trav_check_distance_, 0.3f);
    nh.param("trav_gradient_limit", trav_gradient_limit_, 0.1f);


    // Set up traversability filter
    // grid_map::NormalVectorsFilter<grid_map::GridMap> surface_normal_filter_ = grid_map::NormalVectorsFilter<grid_map::GridMap>();
    // // Configuration of chain filter.
    // if (!surface_normal_filter_.configure()) {
    //     ROS_ERROR("Could not configure the filter chain!");
    // }

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

float WorldModelling::distanceBetweenNodes(cdt_msgs::GraphNode n1, cdt_msgs::GraphNode n2)
{
    float x1 = n1.pose.position.x; 
    float y1 = n1.pose.position.y; 
    float x2 = n2.pose.position.x; 
    float y2 = n2.pose.position.y; 
    
    // ROS_INFO_STREAM("Distance" << std::hypot(x1 - x2, y1 - y2));
    return std::hypot(x1 - x2, y1 - y2); 
}

bool WorldModelling::updateGraph(const float &x, const float &y, const float &theta)
{
    // TODO: you need to update the exploration_graph_ using the current pose

    // You may need to change this flag with some conditions
    bool create_new_node = false;
    float distance_from_last = std::hypot(x - x_last_, y - y_last_);

    if (distance_from_last > node_creation_distance_ || first_node_) {
        create_new_node = true;
        first_node_ = false; // This is to avoid creating more than one nodes
    }

    if (!create_new_node) {
        return false;
    }

    // Create the new node
    cdt_msgs::GraphNode new_node;
    new_node.pose.position.x = x;
    new_node.pose.position.y = y;
    new_node.id.data = num_nodes_; // The id is simply the number of the node
    
    // Update position of last node
    x_last_ = x;
    y_last_ = y;

    // For all nodes other than the first, check for neighbours
    if(!first_node_)
    {
        // Node id for current node to add as neighbour to previous nodes
        std_msgs::Int32 new_node_id;
        new_node_id.data = num_nodes_;

        for (int i = 0; i < num_nodes_; i++) {
            // ROS_INFO_STREAM("Neighbour distance " << neighbor_distance_);
            // ROS_INFO_STREAM("Neighbour path distance " << neighbor_path_distance_ << ", " << new_node.id.data - i << ", " << i);
            if (distanceBetweenNodes(new_node, exploration_graph_.nodes[i]) > neighbor_distance_) 
            {
                continue;
            }
            if (new_node.id.data - i < neighbor_path_distance_)
            {
                continue;
            }

            // Add this node as a neighbour
            std_msgs::Int32 neighbor;
            neighbor.data = i;
            new_node.neighbors_id.push_back(neighbor);  

            // Also update the neighbours of the previous node
            exploration_graph_.nodes[i].neighbors_id.push_back(new_node_id);
        }
    }

    // Finally add the new node to the graph (since all the properties are filled)
    exploration_graph_.nodes.push_back(new_node);

    num_nodes_++; // Increase the number of added nodes

    ROS_INFO("Adding node to graph!!------------------------------------_");

    return true;
}

float WorldModelling::getSlopeAtIndex(const grid_map::Index index) 

{
    auto size = traversability_.getSize();
    const float step = traversability_.getResolution();

    float s_max = 0.0;
    float base_elev = traversability_.at("elevation", index);
    float v1, v2, v3, v4; 

    if (index.x() > 0 && index.x() < size.x() - 1 && index.x() > 0 && index.y() < size.y() - 1) 
    {
        v1 = traversability_.at("elevation", grid_map::Index(index.x() - 1, index.y()));
        v2 = traversability_.at("elevation", grid_map::Index(index.x(), index.y() - 1));
        v3 = traversability_.at("elevation", grid_map::Index(index.x() + 1, index.y()));
        v4 = traversability_.at("elevation", grid_map::Index(index.x(), index.y() + 1));

        s_max = std::max(s_max, std::abs(base_elev - v1));
        s_max = std::max(s_max, std::abs(base_elev - v2));
        s_max = std::max(s_max, std::abs(base_elev - v3));
        s_max = std::max(s_max, std::abs(base_elev - v4));
    }

    // ROS_INFO_STREAM("Slope value: " << s_max / step << ", " << s_max << ", " << step);
    return s_max / step;
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

    // Grid resolution
    const float step = traversability_.getResolution(); // We use the map resolution as the search step

    // Iterate the traversability map to apply a threshold on the height
    for (grid_map::GridMapIterator iterator(traversability_); !iterator.isPastEnd(); ++iterator)
    {
        // We only want to use the valid values
        if (traversability_.isValid(*iterator, "elevation"))
        {

            // TODO Fill the traversability at each position using some criterion based on the other layers
            // How can we figure out if an area is traversable or not?
            // YOu should fill with a 1.0 if it's traversable, and -1.0 in the other case


            // Currently just using a really simple approach - if the elevation is too large, it's not traversable
            float trav_value = 1.0;

            grid_map::Index index = iterator.getUnwrappedIndex();

            int steps = trav_check_distance_ / step;
            int x_start = index.x() - steps/2;
            int y_start = index.y() - steps/2;
            grid_map::Position query_point;
            int xi, yi;

            float min_elev = 1000;
            float max_elev = -1000;
            float max_slope = 0.0;
            float elev;

            // elevation diff in area
            for (int i = 0; i < steps; i++)
            {
                for (int j = 0; j < steps; j++) 
                {
                    xi = x_start + i;
                    yi = y_start + j;
                    index = grid_map::Index(xi, yi);

                    if (!traversability_.isValid(index))
                    {
                        continue;
                    }

                    elev = traversability_.at("elevation", index);

                    // Update slope and min/max elevation 
                    max_slope = std::max(max_slope, getSlopeAtIndex(index));
                    min_elev = std::min(elev, min_elev);
                    max_elev = std::max(elev, max_elev);

                    if (trav_value < 0) break;
                } 

                // If difference of elevations is too large, non-traversable
                // ROS_INFO_STREAM("Max slope and gradient limit" << max_slope << ", " << trav_gradient_limit_);
                if (max_elev - min_elev > 0.4 || max_slope > trav_gradient_limit_) {
                    trav_value = -1.0;
                }

                if (trav_value < 0) break;
            }

            // ROS_INFO_STREAM("Min and max elev: " << min_elev << ", " << max_elev << ", " << trav_value);


            // if (traversability_.at("elevation", *iterator) < elevation_threshold_) {
            //     trav_value = 1.0;
            // }

            traversability_.at("traversability", *iterator) = trav_value;
        }
    }

    traversability_.setBasicLayers({"traversability", "elevation"});
}

void WorldModelling::findCurrentFrontiers(const float &x, const float &y, const float &theta, const ros::Time &time)
{
    // TODO: Here you need to create "frontiers" that denote the edges of the known space
    // They're used to guide robot to new places


    grid_map::Position query_point;

    float theta_deg = theta * 180 / M_PI;
    float range = frontier_search_angle_;
    current_frontiers_.frontiers.clear();

    // All existing frontiers are kept
    // for (auto f : frontiers_.frontiers) {
    //     current_frontiers_.frontiers.push_back(f);
    // }


    for (float angle = theta_deg - range/2; angle < theta_deg + range/2; angle+=frontier_search_angle_resolution_) 
    {
        // The frontiers are expresed in the fixed frame
        geometry_msgs::PointStamped frontier;
        frontier.header.stamp = time;                  // We store the time the frontier was created
        frontier.header.frame_id = input_fixed_frame_; // And the frame it's referenced to
        frontier.point.x = x + max_distance_to_search_frontiers_ * cos(M_PI/180.f * angle);  
        frontier.point.y = y + max_distance_to_search_frontiers_ * sin(M_PI/180.f * angle);


        // Make sure the point is within the map
        query_point = grid_map::Position(frontier.point.x, frontier.point.y);


        if (!traversability_.isInside(query_point))
        {
            continue;
        }

        // Finally we store it in the current frontiers' list
        current_frontiers_.frontiers.push_back(frontier);
    }

    ROS_DEBUG_STREAM("[WorldModelling] Found " << current_frontiers_.frontiers.size() << " new frontiers");
}

int WorldModelling::isTraversable(float x, float y) 
{
    grid_map::Position query_point(x, y);
    if (!traversability_.isInside(query_point)) 
    {
        return 0;
    }

    if (traversability_.atPosition("traversability", query_point) < 0) 
    {
        return -1;
    }
    return 1;
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
    // current_frontiers_.frontiers.clear();

    // Preallocate query point
    grid_map::Position query_point;

    // Filter accumulated frontiers that are too close to current location
    for (auto frontier : frontiers_.frontiers)
    {
        const float &frontier_x = frontier.point.x;
        const float &frontier_y = frontier.point.y;

        // Compute distance to frontier
        float distance_to_frontier = std::hypot(frontier_x - x, frontier_y - y);

        // If point is not traversable, skip
        if (isTraversable(frontier_x, frontier_y) < 0)
        {
            continue;
        }

        // Check the line between the current location and the frontier
        float unit_x = (frontier_x - x)/distance_to_frontier;
        float unit_y = (frontier_y - y)/distance_to_frontier;
        int num_steps = (distance_to_frontier / step) + 1;
        bool is_traversable = true; 
        
        for (int i = 1; i < num_steps; i++) { 
            if (isTraversable(x + unit_x*step*i, y + unit_y*step*i) == -1) {
                is_traversable = false;
                break;
            }
        }
        // // If it's close enough and visible, skip it
        if (is_traversable && distance_to_frontier < distance_to_delete_frontier_)
        {
            continue;
        }
        
        // If the previous test are passed, add frontier to filtered list
        // filtered_frontiers.frontiers.push_back(frontier);
        filtered_frontiers.frontiers.push_back(frontier);
    }

    frontiers_ = filtered_frontiers;

    // Add the current frontiers to the accumulated ones
    for (auto frontier : current_frontiers_.frontiers)
    {
        const float &frontier_x = frontier.point.x;
        const float &frontier_y = frontier.point.y;

        // Compute distance to frontier
        float distance_to_frontier = std::hypot(frontier_x - x, frontier_y - y);

        // If point is not traversable, skip
        if (isTraversable(frontier_x, frontier_y) <= 0)
        {
            continue;
        }

        // Make sure the point isn't too close to any others
        float dist;
        bool too_close = false;
        for (geometry_msgs::PointStamped f : all_frontiers_.frontiers) 
        {
            dist = std::hypot(f.point.x - frontier_x, f.point.y - frontier_y);
            if (dist < frontier_min_separation_) 
            {
                too_close = true;
                break;
            }
        }

        if (too_close) continue;

        // Check the line between the current location and the frontier
        float unit_x = (frontier_x - x)/distance_to_frontier;
        float unit_y = (frontier_y - y)/distance_to_frontier;
        int num_steps = (distance_to_frontier / step) + 1;
        bool is_traversable = true; 
        
        for (int i = 1; i < num_steps; i++) { 
            if (isTraversable(x + unit_x*step*i, y + unit_y*step*i) == -1) {
                is_traversable = false;
                break;
            }
        }

        if (!is_traversable) 
        {
            ROS_INFO_STREAM("Path to frontier is not traversable");
            continue;
        }

        frontiers_.frontiers.push_back(frontier);
        all_frontiers_.frontiers.push_back(frontier);
    }

    // Finally, we update the frontiers using the current ones
    // frontiers_ = current_frontiers_;
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