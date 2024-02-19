#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from icecar.msg import scan_xy
import numpy as np
import csv


rospy.init_node('test')
def cb_scan(data):
    obs = [data.x, data.y]
    ob = np.array([data.x, data.y]).T
    np.savetxt('obs.csv', ob, delimiter=",")
    print("cb obs", np.shape(ob))

rospy.Subscriber('/scan_xy', scan_xy, cb_scan)

rospy.spin()