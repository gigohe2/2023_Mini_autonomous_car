#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
from ultrafastLaneDetector import UltrafastLaneDetector, ModelType
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time


class YOLO_cam():
    def __init__(self, loop_hz=20):
        self.rate = rospy.Rate(loop_hz)


class Lane_detection():
    def __init__(self, loop_hz=20):
        self.rate = rospy.Rate(loop_hz)
        self.model_path = "/home/icelab_nx/catkin_ws/src/icecar/src/lane_detection/DL/models/tusimple_18.pth"
        #self.model_path = "/home/icelab_nx/catkin_ws/src/icecar/src/lane_detection/models/culane_18.pth"
        self.model_type = ModelType.TUSIMPLE
        self.use_gpu = True
        self.lane_detector = UltrafastLaneDetector(self.model_path, self.model_type, self.use_gpu)
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.cb_cam)
        self.img = None
        self.cte = 0
        rospy.wait_for_message('/camera/color/image_raw', Image)


    def cb_cam(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def post_process(self):
        t = time.time()
        output_img, self.cte = self.lane_detector.detect_lanes(self.img)
        print(self.cte, ",process :", time.time()-t)
        cv2.imshow('output', output_img)
        cv2.waitKey(1)


    
if __name__ == '__main__':
    rospy.init_node('lanedetector')
    lane = Lane_detection()
    while not rospy.is_shutdown():
        lane.post_process()
        lane.rate.sleep()
