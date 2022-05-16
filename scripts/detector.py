#!/usr/bin/env python
    
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from getfrontier import getfrontier
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

import sys
sys.setrecursionlimit(10**7)

mapData = OccupancyGrid()

def mapCallback(data):
    global mapData
    global map_topic

    mapData = data
    rospy.loginfo(rospy.get_caller_id() + "// " + map_topic + " map data is currently received")
    
def detector_node():
    global mapData

    #--------- init ---------------
    rospy.init_node('detector', anonymous=True)
    rospy.loginfo_once("---- 'detector' node is loaded. ----")
    #------------------------------
    
    #--------- subscriber ------
    # http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    # http://wiki.ros.org/rospy/Overview/Parameter%20Server

    global map_topic
    map_topic = rospy.get_param('~map_topic')
    rospy.loginfo_once("----- Requested map topic is " + map_topic + " -----")

    # rospy.Publisher(topic_name, msg_class, queue_size)
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallback)
    #------------------------------
    
    #--------- Publisher ----------
    marker_pub = rospy.Publisher("marker", Marker, queue_size=10) # marker publish
    #------------------------------

    while mapData.header.seq<1 or len(mapData.data)<1:
        pass

    rate = rospy.Rate(50)

    #----- rviz visualization -----
    # https://velog.io/@717lumos/Rviz-Rviz-%EC%8B%9C%EA%B0%81%ED%99%94%ED%95%98%EA%B8%B0-Marker
    exploration_goal = PointStamped()
    #------------------------------


    while not rospy.is_shutdown():
        #----- rviz visualization -----
        frontier_points = Marker()

        frontier_points.header.frame_id = mapData.header.frame_id
        frontier_points.header.stamp=rospy.Time.now()
        frontier_points.ns = "points"
        frontier_points.id = 0
        
        frontier_points.type = Marker.POINTS
        frontier_points.action = Marker.ADD

        frontier_points.scale.x = 0.2
        frontier_points.scale.y = 0.2
        frontier_points.color = ColorRGBA(1, 1, 0, 1)
        frontier_points.lifetime == rospy.Duration()
        #------------------------------

        # getfrontier Node
        frontiers = getfrontier(mapData)

        for i in range(len(frontiers)):
            x=frontiers[i]
            exploration_goal.header.frame_id = mapData.header.frame_id
            exploration_goal.header.stamp = rospy.Time(0)
            exploration_goal.point.x = x[0]
            exploration_goal.point.y = x[1]
            exploration_goal.point.z = 0
            frontier_points.points = [exploration_goal.point]

            frontier_points.points=[exploration_goal.point]
        marker_pub.publish(frontier_points)
            
    
        rate.sleep()

if __name__ == '__main__':
    try:
        detector_node()
    except rospy.ROSInterruptException:
        pass