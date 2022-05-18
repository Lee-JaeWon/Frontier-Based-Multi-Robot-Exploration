#!/usr/bin/env python

import rospy

import numpy as np
import cv2
from copy import copy
from matplotlib import pyplot as plt

def getfrontier(mapData):
    rospy.loginfo_once("---- 'getfrontier' node is loaded ----")
    rospy.loginfo_once("OpenCV Version : " + cv2.__version__)

    data = mapData.data

    # http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    width = mapData.info.width
    height = mapData.info.height
    resolution = mapData.info.resolution
    startx = mapData.info.origin.position.x
    starty = mapData.info.origin.position.y 

    # print("startx is ",startx)
    # print("starty is ",starty)

    img = np.zeros((height, width, 1), np.uint8)

    for i in range(0, height):
        for j in range(0, width):
            if data[i * width + j] == 100:
                img[i, j] = 0
            elif data[i * width + j] == 0:
                img[i, j] = 100
            elif data[i * width + j] == -1:
                img[i, j] = 50

    map_bin = cv2.inRange(img, 0, 1)
    edges = cv2.Canny(img, 100, 200)

    # plt.imshow(edges, cmap='gray', origin='lower')
    # plt.draw()
    # plt.pause(0.001)

    _, contours, _ = cv2.findContours(map_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(map_bin, contours, -1, (255,255,255), 5)

    map_bin=cv2.bitwise_not(map_bin)
    res = cv2.bitwise_and(map_bin, edges)

    frontier = copy(res) # Shallow Copy : new id

    _, contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

    _, contours, _ = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    #------ frontier show ------
    plt.imshow(frontier, cmap='gray', origin='lower')
    plt.draw()
    plt.pause(0.001)

    all_pts = []

    if len(contours)>0:
        i=0
        
        for i in range(0,len(contours)):
            cnt = contours[i]

            M = cv2.moments(cnt)

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # print("cx is", cx)
            # print("cy is",cy)

            xr = cx*resolution+startx
            yr = cy*resolution+starty
            # print("xr is", xr)
            # print("yr is",yr)

            pt = [np.array([xr,yr])]

            if len(all_pts)>0:
                all_pts=np.vstack([all_pts,pt])
            else:
                all_pts=pt

    return all_pts