#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
from pyproj import Transformer
from tf.transformations import euler_from_quaternion
import rospy
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from util.Util import Carstatus
import os
from ref_tracker import RefTracker, Gain
from icecar.msg import car_status

username = os.environ.get('USER') or os.environ.get('USERNAME')

class GpsDriver():

    def __init__(self):
        
        rospy.init_node('icelab')
        rospy.Subscriber('/filter/positionlla', Vector3Stamped, self.position_callback)
        rospy.Subscriber('/filter/quaternion', QuaternionStamped, self.quat_callback)
        self.rate = rospy.Rate(50)
        self.gps_pub = rospy.Publisher('/icelab/gpsdrive', Float32MultiArray, queue_size=10)
        self.transformer = Transformer.from_crs('epsg:4326', 'epsg:5178')
        self.my_car = Carstatus()
        self.ref_tracker = RefTracker(gain_lowspeed=Gain(Kp=0.25, Kd=0.01, Kpa=0.),
                                    gain_highspeed=Gain(Kp=0.1, Kd=0.01, Kpa=0.1),
                                    look_ahead_dist=2,
                                    dt=1/50)
        self.gpsdrive_info = Float32MultiArray()
        self.my_car.yaw = 0
        # rospy.wait_for_message('/filter/positionlla', Vector3Stamped)
        # rospy.wait_for_message('/filter/quaternion', QuaternionStamped)
        

    def position_callback(self, data):
        self.my_car.x, self.my_car.y = self.transformer.transform(data.vector.x, data.vector.y)

    def normalize_pi(self, data):
        if data > np.pi:
            data = -2*np.pi + data
        elif data < -np.pi:
            data = 2*np.pi + data
        return data

    def quat_callback(self, data):
        quaternion = (
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
            data.quaternion.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        heading = self.normalize_pi(- euler_angles[2] + np.pi/2) # radian
        self.my_car.yaw = 2*np.pi + heading if heading < 0 else heading

    def gps_publisher(self, steer, speed, min_idx):
        self.gpsdrive_info.data = [steer, speed, min_idx]
        self.gps_pub.publish(self.gpsdrive_info)
        
    
    def main(self):
        self.ref_tracker.set_ref_path('utm-k_path')
        self.ref_tracker.set_velocity_profile('velocity_profile_1')
        #rospy.loginfo("set path")
        while not rospy.is_shutdown():
            steer, speed, idx = self.ref_tracker.do(self.my_car)

            self.gps_publisher(steer, speed, idx)
            self.rate.sleep()

if __name__ == '__main__':
    gps_driver = GpsDriver()

    gps_driver.main()
