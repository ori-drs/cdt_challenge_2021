#include <ros/ros.h>
#include <world_modelling_cdt/world_modelling.h>

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "world_modelling");
    // Init node handler
    ros::NodeHandle nh("~");
    // Init world modelling
    WorldModelling modelling(nh);
    // Spin to process the incoming messages
    ros::spin();
}