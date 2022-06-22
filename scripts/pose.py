#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print(msg.pose.pose)

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('robot_1/odom', Odometry, callback)
rospy.spin()