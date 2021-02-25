#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include <eigen_conversions/eigen_msg.h>

using namespace std;


class Pass{
  public:
    Pass(ros::NodeHandle node_);

    ~Pass(){
    }
  private:
    ros::NodeHandle node_;

    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;

    ros::Subscriber poseSub_;
    void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    ros::Subscriber cloudSub_;
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
    ros::Publisher cloudPub_;
};

Pass::Pass(ros::NodeHandle node_){
  //cloudSub_     = node_.subscribe(std::string("/os_cloud_node/points"), 100, &Pass::cloudHandler, this);
  cloudSub_     = node_.subscribe(std::string("/vilens/point_cloud_transformed_processed"), 100, &Pass::cloudHandler, this);
  poseSub_     = node_.subscribe(std::string("/vilens/pose"), 100, &Pass::poseHandler, this);
  cloudPub_ = node_.advertise<sensor_msgs::PointCloud2>("/vilens/point_cloud_transformed_processed_repub", 10);
}

void Pass::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
  std::cout << "got cloud\n";
  sensor_msgs::PointCloud2 copy = *msg;
  copy.header.frame_id = "base_link";
  cloudPub_.publish(copy);
}

void Pass::poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  std::cout << "got pose\n";
  ros::Time msg_time(msg->header.stamp.sec, msg->header.stamp.nsec);

  tf::StampedTransform base_to_odom_tf;
  Eigen::Isometry3d base_to_odom;
  try {
    listener_.waitForTransform("odom", "base_link", msg_time, ros::Duration(1.0));
    listener_.lookupTransform("odom", "base_link", msg_time, base_to_odom_tf);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s : ", ex.what());
    ROS_ERROR("Skipping pose.");
    return;
  }
  tf::transformTFToEigen(base_to_odom_tf, base_to_odom);

  Eigen::Isometry3d base_to_vilens;
  tf::Pose temp_tf_pose;
  tf::poseMsgToTF(msg->pose.pose, temp_tf_pose);
  tf::transformTFToEigen(temp_tf_pose, base_to_vilens);

  Eigen::Isometry3d vilens_to_odom = base_to_odom * base_to_vilens.inverse();
  tf::Transform vilens_to_odom_tf;
  tf::transformEigenToTF (vilens_to_odom, vilens_to_odom_tf);

  broadcaster_.sendTransform(tf::StampedTransform(vilens_to_odom_tf, msg_time, "odom", "odom_vilens"));
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "nav_bridge");
  ros::NodeHandle nh;
  Pass app(nh);
  cout << "Ready to follow position" << endl << "============================" << endl;
  ros::spin();
  return 0;
}
