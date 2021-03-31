#include <ros/ros.h>
#include <object_detector_cdt/object_detector.h>

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "object_detector");
    // Init node handler
    ros::NodeHandle nh("~");
    // Init object detector
    ObjectDetector detector(nh);
    // Spin to process the incoming messages
    ros::spin();
}
