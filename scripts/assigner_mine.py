#!/usr/bin/env python

from copy import copy
from unicodedata import name
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from numpy import array

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf

# custom msg
from frontier_based_exploration.msg import PointArray

mapData=OccupancyGrid()
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()

frontiers = []

def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data



def assign_node():
    global frontiers,mapData,global1,global2,global3
    rospy.init_node('assigner', anonymous=False)

    # Parameter
    map_topic= rospy.get_param('~map_topic','map')
    frontiers_topic = rospy.get_param('~frontiers_topic','/detected_points')
    rateHz = rospy.get_param('~rate',100)
    namespace = rospy.get_param('~namespace','')
    rate = rospy.Rate(rateHz)

    # Subscriber
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)

    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    # Check
    rospy.loginfo_once(map_topic)
    rospy.loginfo_once(namespace)
    # [INFO] [1655881627.511653, 0.000000]: /robot_2/map
    # [INFO] [1655881627.514883, 0.000000]: robot_2

   

    # Safety
    while (len(mapData.data)<1):
        rospy.loginfo_once("Current No Map Data")
        pass
    while len(frontiers)<1:
        rospy.loginfo_once("Current No frontier")
        pass

    rospy.loginfo_once(mapData.header.frame_id)
#     [INFO] [1655880074.805074, 26.682000]: robot_3/map
#     [INFO] [1655880074.805129, 26.682000]: robot_2/map
#     [INFO] [1655880074.805211, 26.682000]: robot_1/map
    # For get current position

    listener = tf.TransformListener()
    (trans, rot) = listener.waitForTransform(namespace+'/'+'map', namespace+'/'+'base_link', rospy.Time(0), rospy.Duration(10.0))
    
    while not rospy.is_shutdown():
        assigned_point = []
        centroids = copy(frontiers)

         

        (trans, rot) = listener.lookupTransform(namespace+'/'+'map', namespace+'/'+'base_link', rospy.Time(0))
        # (trans, rot) = listener.lookupTransform(namespace+'/'+'map', namespace+'/'+'base_link', rospy.Time(0))
        print(trans[0])
        print(trans[1])

        rate.sleep()

if __name__ == '__main__':
    try:
        assign_node()
    except rospy.ROSInterruptException:
        pass