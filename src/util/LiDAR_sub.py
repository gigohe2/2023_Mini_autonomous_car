#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
from icecar.msg import scan_xy

class LiDAR_2D():
    def __init__(self, loop_hz=10):
        self.rate = rospy.Rate(loop_hz)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)
        self.xy_pub = rospy.Publisher('scan_xy', scan_xy, queue_size=1)
        self.scan_data = None
        self.angle_increment = 0.005482709966599941
        self.min_angle = -3.1415927410125732
        self.max_angle = 3.1415927410125732
        self.xy = None
        self.scan_xy = scan_xy()
        self.angles = []
        self.init_values()
        rospy.wait_for_message('scan', LaserScan)

    def init_values(self):
        angle = self.min_angle
        for i in range(0, 1147):
            tmp = [np.cos(angle), np.sin(angle)]
            self.angles.append(tmp)
            angle += self.angle_increment
        self.angles = np.array(self.angles)

    def scan_cb(self, data):
        self.scan_data = data

    def visualize(self):
        plt.clf()
        plt.scatter(self.xy[0,:], self.xy[1,:], 0.1)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim([-5, 5])
        plt.ylim([0, 12])
        plt.pause(0.0001)


    def data_handler(self):
        st = time.time()
        
        y = -np.multiply(self.scan_data.ranges, self.angles[:,0])
        x = np.multiply(self.scan_data.ranges, self.angles[:,1])

        xy = np.array([x, y])
        self.xy = xy[:, np.isfinite(xy).all(axis=0)]

        self.scan_xy.y = self.xy[0]
        self.scan_xy.x = self.xy[1]

        idx = np.where((self.xy[0,:] < 5) & (self.xy[0,:] > -5) & (self.xy[1,:] > 0))
        self.xy = self.xy[:,idx]
        self.xy_pub.publish(self.scan_xy)
        #self.xy_pub.publish(self.xy)
        print("processing time:", time.time()-st)
        


if __name__=='__main__':
    rospy.init_node("lidar")
    lidar = LiDAR_2D()
    plt.figure()
    
    plt.ion()
    plt.show()
    while not rospy.is_shutdown():
        lidar.data_handler()
        lidar.visualize()
        lidar.rate.sleep()
