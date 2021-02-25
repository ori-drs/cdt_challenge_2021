#!/usr/bin/env python
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

def callback(msg):
    # publish 
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    pose = (position.x, position.y, position.z )
    ori = (orientation.x, orientation.y, orientation.z, orientation.w)
    br = tf.TransformBroadcaster()
    br.sendTransform(pose, ori, rospy.Time.now(), "base_link", "odom")
    # publish map frame
    #br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "map", "odom")

    


if __name__ == '__main__':
    rospy.init_node("AddFrame")
    rospy.Subscriber("/odometry/filtered", Odometry, callback)
    rospy.spin()
