#!/usr/bin/env python
import rospy
from cdt_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

statePub = rospy.Publisher('/state_estimator/robot_state', RobotState, queue_size=10)
posePub = rospy.Publisher('/state_estimator/pose_in_odom', PoseWithCovarianceStamped, queue_size=10)

def state_callback(msg):
    stateMsg = RobotState()
    stateMsg.twist.twist = msg.twist.twist
    stateMsg.twist.header = msg.header
    stateMsg.pose.pose = msg.pose.pose
    stateMsg.pose.header = msg.header
    statePub.publish(stateMsg)

    poseMsg = PoseWithCovarianceStamped()
    poseMsg.header = msg.header
    poseMsg.pose = msg.pose
    posePub.publish(poseMsg)

if __name__ == '__main__':
    rospy.init_node('jackal_state_publisher')

    # /velocity_controller/odom
    #odom_topic = rospy.get_param('~odom_topic', '/odom')
    rospy.Subscriber("/odometry/filtered", Odometry, state_callback, queue_size=10)

    rospy.spin()
