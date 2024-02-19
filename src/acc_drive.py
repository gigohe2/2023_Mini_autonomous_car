#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32
from util.Util import PID
from Kalman import KalmanRelativeVelocity, KalmanPos2Vel
import time

class ACC():
    def __init__(self, loop_hz=20):
        self.rate = rospy.Rate(loop_hz)
        self.hunter_info_sub = rospy.Subscriber('/icelab/yolo/hunter_info', Float32MultiArray, self.cb_hunter)
        self.is_hunter_sub = rospy.Subscriber('/icelab/yolo/is_hunter', Int32, self.cb_flag)
        self.accdrive_pub = rospy.Publisher('/icelab/accdrive', Float32MultiArray, queue_size=1)
        self.mode_pub = rospy.Publisher('/icelab/mode_change', Int32, queue_size=1)
        self.is_hunter, self.acc_done = False, False
        self.hunter_dist, self.hunter_cte = 0, 0
        self.accdrive_info = Float32MultiArray()
        self.ref_dist, self.prev_dist, self.speed = 5, 0, 0 # [m], [m/s]
        self.speed_PID = PID(0.1, 0, 0.001, 1/loop_hz, 1)
        self.kf = KalmanRelativeVelocity(initial_velocity=0, process_noise=1e-2, measurement_noise=1e-1)
        self.kf_dist = KalmanPos2Vel(P0 = np.array([[1,0],[0,1]]), x0=np.array([[0],[0]]))
        self.stop_time, self.prev_time = 0, 0
        self.hunter_steer = 0.5
    

    def cb_flag(self, msg):
        self.is_hunter = bool(msg.data)

    def cb_hunter(self, msg):
        self.hunter_dist = msg.data[0]
        self.hunter_cte  = msg.data[1]

    def avoidance(self):
        # turn right and go staright 
        rate = rospy.Rate(50)
        self.accdrive_info.data = [0.8, 1, 0]
        for i in range(0, 75):
            self.accdrive_pub.publish(self.accdrive_info)
            rate.sleep()

        self.accdrive_info.data = [0.2, 1, 0]
        for i in range(0, 75):
            self.accdrive_pub.publish(self.accdrive_info)
            rate.sleep()

        self.accdrive_info.data = [0.5, 1, 0]
        for i in range(0, 100):
            self.accdrive_pub.publish(self.accdrive_info)
            rate.sleep()
        self.accdrive_info.data = [0.5, 0, 0]
        self.acc_done = True
        self.mode_pub.publish(1)
        exit()
        
    def control(self):
        if self.is_hunter and (not self.acc_done):
            dist_err = self.hunter_dist - self.ref_dist
            if False: # relative velocity estimation
                dist_rate = (dist_err - self.prev_dist) / 20
                rel_vel = self.kf.update((self.speed+dist_rate), 1/20)
                self.speed = np.clip(self.speed+rel_vel, 0, 1)

            elif True: # kf - Pos2Vel
                kf_speed, _ = self.kf_dist.update(dist_err, 0.05)
                kf_speed = np.clip(kf_speed, -0.1, 0.1) 
                self.speed = np.clip(self.speed + kf_speed, 0, 5)
                #print("dist : {0}, err : {1}, kf_speed : {2}, speed : {3}".format(self.hunter_dist, dist_err, kf_speed, self.speed))
            else: # dist PID
                rel_speed = self.speed_PID.do(dist_err)
                self.speed = np.clip(self.speed + rel_speed, 0, 1.5)
                print("rel_speed : {0}, speed : {1}".format(rel_speed, self.speed))

            # stop and avoidance
            if self.speed < 0.2 :
                self.stop_time += time.time() - self.prev_time
                #print(self.stop_time)
                if self.stop_time > 2:
                    rospy.loginfo("start avoiding hunter")
                    self.avoidance()
                    
            else:
                self.stop_time = 0

            self.hunter_steer = np.interp(self.hunter_cte, [-640, 640], [0,1])
            self.accdrive_info.data = [self.hunter_steer, self.speed, self.is_hunter]
            self.accdrive_pub.publish(self.accdrive_info)
        else:
            self.stop_time = 0
            self.accdrive_info.data = [self.hunter_steer, self.speed, 0]
            self.accdrive_pub.publish(self.accdrive_info)
        self.prev_time = time.time()

            
if __name__=='__main__':
    rospy.init_node("ACC")
    acc = ACC()
    acc.prev_time = time.time()
    while not rospy.is_shutdown():
        
        acc.control()
        acc.rate.sleep()