#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64, Int32
from xsens_mti_driver.msg import XsStatusWord

rospy.init_node('voltage_publisher')

volt = 0
rtk_status = 0

def cb_vesc(data):
    global volt
    volt = data.state.voltage_input

def cb_rtk(data):
    global rtk_status
    rtk_status = int(data.rtk_status)    
    
rospy.Subscriber('/sensors/core', VescStateStamped, cb_vesc)
rospy.Subscriber('/status', XsStatusWord, cb_rtk)
voltage_pub = rospy.Publisher('/voltage', Float64, queue_size=1)
rtk_pub = rospy.Publisher('/rtk_status', Int32, queue_size=1)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    voltage_pub.publish(volt)
    rtk_pub.publish(rtk_status)
    rate.sleep()

