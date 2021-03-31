#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String

team_name_pub         = rospy.Publisher('/team_name', String, queue_size=10)
team_member_names_pub = rospy.Publisher('/team_member_names', String, queue_size=10)

if __name__ == '__main__':
    rospy.init_node('cdt_publish_team_names')

    # Name
    team_name = rospy.get_param('~team_name', 'Set your team name please.')
    team_member_names = rospy.get_param('~team_member_names', 'Set your member names please.')    
   
    while not rospy.is_shutdown():
        team_member_names_pub.publish(team_member_names)
        team_name_pub.publish(team_name)
        rospy.sleep(5.0)
