#!/usr/bin/env python
import roslib
import rospy
import tf
from cdt_msgs.msg import RobotState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped

statePub = rospy.Publisher('/state_estimator/robot_state', RobotState, queue_size=10)
posePub = rospy.Publisher('/state_estimator/pose_in_odom', PoseWithCovarianceStamped, queue_size=10)
br = tf.TransformBroadcaster()

def callback(msg):
    try:
        # Look for message
        idx = msg.name.index("jackal::base_link")

        # Get pose
        position    = msg.pose[idx].position
        orientation = msg.pose[idx].orientation

        time = rospy.Time.now()

        # publish Tf
        pos = (position.x, position.y, position.z )
        ori = (orientation.x, orientation.y, orientation.z, orientation.w)        
        br.sendTransform(pos, ori, time, "base_link", "odom")

        # Publish robot state
        poseMsg = PoseWithCovarianceStamped()
        poseMsg.header.stamp     = time
        poseMsg.header.frame_id  = "odom"
        poseMsg.pose.pose.position    = position
        poseMsg.pose.pose.orientation = orientation
        posePub.publish(poseMsg)

        stateMsg = RobotState()
        stateMsg.pose.pose.position    = position
        stateMsg.pose.pose.orientation = orientation
        stateMsg.pose.header.stamp     = time
        stateMsg.pose.header.frame_id  = "odom"
        statePub.publish(stateMsg)

    except Exception as e:
        pass
        #print(e)

if __name__ == '__main__':
    rospy.init_node("jackal_state_publisher")
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
    rospy.spin()
