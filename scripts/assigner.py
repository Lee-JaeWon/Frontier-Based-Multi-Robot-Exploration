#!/usr/bin/env python

from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from numpy import array

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

    frontiers_topic = rospy.get_param('~frontiers_topic','/detected_points')
    rateHz = rospy.get_param('~rate',100)

    rate = rospy.Rate(rateHz)

    rospy.Subscriber(frontiers_topic, PointArray, callBack)

    while (len(mapData.data)<1):
        rospy.loginfo_once("Current No Map Data")
        pass
    while len(frontiers)<1:
        rospy.loginfo_once("Current No frontier")
        pass

    while not rospy.is_shutdown():
        centroids = copy(frontiers)
        print(centroids)

if __name__ == '__main__':
    try:
        assign_node()
    except rospy.ROSInterruptException:
        pass