#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from vesc_msgs.msg import VescStateStamped
import rospy
from std_msgs.msg import Float64, Int32, Float32MultiArray, String, Float32
import time

LANE_START_IDX = 1000
LANE_END_IDX   = 2000

DRIVE_MANUAL = 0
DRIVE_GPS    = 1
DRIVE_ACC    = 2
DRIVE_LANE   = 3

MODES = {
    0: "DRIVE_MANUAL",
    1: "DRIVE_GPS",
    2: "DRIVE_ACC",
    3: "DRIVE_LANE"
}

class VESC_control():

    def __init__(self):
        rospy.init_node('VESC')
        self.rate = rospy.Rate(50)
        rospy.Subscriber('icejoy', Float32MultiArray, self.cb_receiver)
        rospy.Subscriber('/sensors/core', VescStateStamped, self.cb_vesc)
        rospy.Subscriber('/icelab/gpsdrive', Float32MultiArray, self.cb_gpsdrive)
        rospy.Subscriber('/icelab/accdrive', Float32MultiArray, self.cb_accdrive)
        rospy.Subscriber('/icelab/mode_change', Int32, self.cb_mode_change)
        rospy.Subscriber('/person', String, self.cb_person) # tf_cls publish
        rospy.Subscriber('/icelab/lane_steer', Float64, self.cb_lane)
        self.mode_pub = rospy.Publisher('/icelab/driving_mode', Int32, queue_size=10)
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=10) # 0(좌회전) ~ 1(우회전)
        self.log_steering_pub = rospy.Publisher('/icelab/steer', Float64, queue_size=10) # 0(좌회전) ~ 1(우회전)
        self.speed_pub = rospy.Publisher('/icelab/speed', Float64, queue_size=10)
        self.volt, self.driving_mode, self.mode_fix = 0, rospy.get_param('~mode', 1), bool(False)
        self.old_angle, self.old_speed = 0.5, 0
        self.manual_angle, self.manual_speed = 0.5, 0
        self.gps_angle, self.gps_speed, self.gps_minidx = 0.5, 0, 0
        self.acc_angle, self.acc_speed, self.is_hunter, self.prev_hunter, self.st_hunter, self.acc_flag = 0.5, 0, False, False, 0, False
        self.time, self.time_tmp = None, 0
        self.lane_steer = Float32()
        self.person = String()
        rospy.loginfo("driving mode : %s", MODES[self.driving_mode])

    def cb_lane(self, msg):
        self.lane_steer = msg.data
        

    def cb_person(self, msg):
        self.person = msg.data

    def cb_mode_change(self, msg):
        self.driving_mode = msg.data
        if self.driving_mode == 0:
            self.mode_fix = True
        rospy.loginfo("Change driving mode to : %s", MODES[self.driving_mode])

    def cb_gpsdrive(self, msg):
        self.gps_angle = msg.data[0]
        self.gps_speed = msg.data[1]
        self.gps_minidx = msg.data[2]
     


    def cb_accdrive(self, msg):
        self.acc_angle = msg.data[0]
        self.acc_speed = msg.data[1]
        self.is_hunter = msg.data[2]
        #print(self.is_hunter, self.prev_hunter, self.acc_flag)
        if self.is_hunter and (not self.prev_hunter) and (not self.acc_flag) :
            self.st_hunter = time.time() 
        elif self.is_hunter and (not self.acc_flag):
            if (time.time() - self.st_hunter) > 1:
                self.driving_mode = DRIVE_ACC
                self.acc_flag = True
                rospy.loginfo("Change driving mode to : %s", MODES[self.driving_mode])
        self.prev_hunter = self.is_hunter

    def cb_vesc(self, data):
        self.volt = data.state.voltage_input

    def cb_receiver(self,data):
        self.manual_angle = np.interp(data.data[1], [1000,1960], [0,1])
        self.manual_speed = -np.interp(data.data[0],[1000,2080], [-20000,20000] ) #1540
        if abs(self.manual_speed)<2000:
            self.manual_speed = 0
        if (self.old_angle < 0.05 and self.manual_angle < 0.05) and (self.old_speed > 25000 and self.manual_speed):
            self.time_tmp += (rospy.get_time()-self.time)
            if self.time_tmp > 1:
                self.time_tmp = 0
                self.driving_mode, self.mode_fix = DRIVE_MANUAL, True
                rospy.loginfo("User interrupt : set driving_mode to %s", MODES[self.driving_mode])
        else:
            self.time_tmp = 0
        self.old_angle = self.manual_angle
        self.old_speed = self.manual_speed
        self.time = rospy.get_time()
    
    def speed2RPM(self, speed):
        return (speed * 60 * 20 / (2 * np.pi * 55 * 10**(-3)))

    def RPM2speed(self, RPM):
        return (1/self.speed2RPM(1/(RPM+1e-10)))
    
    def motor_publish(self, angle, speed):
        self.steering_pub.publish(Float64(angle))
        self.motor_pub.publish(Float64(speed))
        self.speed_pub.publish((speed/20) * 2 * np.pi * 55 * 10**(-3) / 60)
    
    def main(self):
        while not rospy.is_shutdown():
            if self.driving_mode == DRIVE_MANUAL:
                steer, rpm = self.manual_angle, self.manual_speed
                
            elif self.driving_mode == DRIVE_GPS:
                if self.person == "slow":
                    steer, rpm, idx = self.gps_angle, -self.speed2RPM(self.gps_speed)/2, self.gps_minidx
                elif self.person == "stop":
                    steer, rpm, idx = self.gps_angle, 0, self.gps_minidx
                else:
                    steer, rpm, idx = self.gps_angle, -self.speed2RPM(self.gps_speed), self.gps_minidx
        
            elif self.driving_mode == DRIVE_ACC:
                steer, rpm = self.acc_angle, -self.speed2RPM(self.acc_speed)
                #print("acc_speed :", self.acc_speed)

            elif self.driving_mode == DRIVE_LANE:
                # - : 좌측 / + : 우측
                steer, rpm = self.lane_steer, -self.speed2RPM(self.gps_speed)
            else:
                steer, rpm = 0.5, 0
            rospy.loginfo('mode: {}, steer: {}, speed: {:.2f}'.format(MODES[self.driving_mode], steer, self.RPM2speed(-rpm)))
            self.motor_publish(steer, self.manual_speed)
            self.mode_pub.publish(self.driving_mode)
            self.rate.sleep()

if __name__ == '__main__':
    vesc = VESC_control()
    vesc.main()
