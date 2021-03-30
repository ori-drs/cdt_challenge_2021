#include <ros/ros.h>
#include <position_controller_cdt/position_controller.h>

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "position_controller");
    // Init node handler
    ros::NodeHandle nh("~");
    // Init position controller
    PositionController controller(nh);
    // Spin to process the incoming messages
    ros::spin();
}