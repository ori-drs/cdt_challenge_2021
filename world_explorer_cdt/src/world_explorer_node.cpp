#include <ros/ros.h>
#include <world_explorer_cdt/world_explorer.h>

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "world_explorer");
    // Init node handler
    ros::NodeHandle nh("~");
    // Init world explorer
    WorldExplorer explorer(nh);
    // Spin to process the incoming messages
    ros::spin();
}