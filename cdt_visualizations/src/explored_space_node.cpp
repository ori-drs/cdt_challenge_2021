#include <ros/ros.h>
#include <cdt_visualizations/explored_space_vis.h>

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "explored_space_vis");
    // Init node handler
    ros::NodeHandle nh("~");
    // Init world modelling
    ExploredSpace vis(nh);
    // Spin to process the incoming messages
    ros::spin();
}