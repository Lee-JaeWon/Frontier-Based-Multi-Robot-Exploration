#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

rospy.init_node("rviz_marker")
marker_pub = rospy.Publisher("marker", Marker, queue_size=1)
text_pub = rospy.Publisher("text", Marker, queue_size=1)

while not rospy.is_shutdown():
    rviz_points = Marker()

    rviz_points.header.frame_id = "/my_frame"
    rviz_points.ns = "points"
    rviz_points.id = 1

    rviz_points.type = Marker.POINTS
    rviz_points.action = Marker.ADD

    rviz_points.color = ColorRGBA(1, 1, 0, 1)
    rviz_points.scale.x = 0.3
    rviz_points.scale.y = 0.3

    # rviz_points.points.append(Point(1, 1, 0))     # 점 하나만 넣기

    for i in range(0, 5):
        rviz_points.points.append(Point(i, i, 0))

    marker_pub.publish(rviz_points)