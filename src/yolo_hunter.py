#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ultralytics import YOLO
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32, String
from cv_bridge import CvBridge
import time
from sklearn.cluster import KMeans


class YOLO_cam():
    def __init__(self, loop_hz=30):
        self.rate = rospy.Rate(loop_hz)
        # self.model = YOLO('/home/icelab_nx/catkin_ws/src/icecar/src/yolo/best_s.pt')
        self.model = YOLO('/home/icelab_nx/catkin_ws/src/icecar/src/yolo/best_m-01-21.pt')
        self.cam_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.cb_cam)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.cb_depth)
        self.cls_pub = rospy.Publisher('/icelab/yolo/classes', Float32MultiArray, queue_size=1)
        self.hunter_info_pub = rospy.Publisher('/icelab/yolo/hunter_info', Float32MultiArray, queue_size=1)
        self.is_hunter_pub = rospy.Publisher('/icelab/yolo/is_hunter', Int32, queue_size=1)
        self.person_pub = rospy.Publisher('/person', String, queue_size=1) # tf_cls publish
        self.bridge = CvBridge()
        self.img = None
        self.depth = None
        self.depth_resolution = 0.001 # 16-bit int 1[mm] <-> m scale
        self.cls = Float32MultiArray()
        self.hunter_region = Float32MultiArray()
        self.hunter_info = Float32MultiArray()
        self.msg = String()
        self.msg.data = 'go'
        self.kmeans = KMeans(n_clusters=3, random_state=0, n_init="auto")
        self.dt = 0.08
        
        rospy.wait_for_message('/camera/color/image_raw', Image)
        #rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)

    def cb_cam(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #print("rgb : ", np.shape(self.img))

    def cb_depth(self, data):
        self.depth = self.bridge.imgmsg_to_cv2(data, "passthrough")
        #print("depth : ", np.shape(self.depth))


    def post_process(self, visualize=True):
        t = time.time()
        self.results = self.model(self.img, conf=0.5, verbose=False)

        for result in self.results:
            # if len(result.boxes.xywh.cpu().numpy()):
            self.cls.data = result.boxes.cls.cpu().numpy()
            #print(self.cls.data)
            # else:
            #     self.cls.data = list()

            for box in result.boxes:
                if not box.cls: # -> hunter
                    self.hunter_region.data = box.xyxy[0].cpu().numpy()
                    x1,y1,x2,y2 = map(int, self.hunter_region.data)
                
                    hunter_depth = self.depth[y1:y2, x1:x2]
                    lin_depth = hunter_depth.reshape(-1, 1)
                    lin_cluster = self.kmeans.fit_predict(lin_depth)
                    lin_depth = np.squeeze(lin_depth, 1)

                    cluster_counts = np.bincount(lin_cluster)
                    largest_cluster_idx = np.argmax(cluster_counts)
                    mean_dist = np.mean(lin_depth[lin_cluster == largest_cluster_idx]) * self.depth_resolution

                    center_err = (x2+x1)/2 - 640
                    self.hunter_info.data = [mean_dist, center_err]
                    self.hunter_info_pub.publish(self.hunter_info)
                    self.is_hunter_pub.publish(True)
                    self.msg.data = "go"
                else:
                    size = box.xywh.cpu().numpy()
                    if len(size) != 0:
                        x = size[0][0]
                        y = size[0][1]
                        w = size[0][2]
                        h = size[0][3]
                        #print(x, y, w, h)
                        if (x <840 and x > 440) and w > 60:
                            self.msg.data = 'stop'
                        elif (x < 1000 and x > 280) and w > 30:
                            self.msg.data = 'slow'
                        else:
                            self.msg.data = 'go'
                    self.is_hunter_pub.publish(False)
                    self.person_pub.publish(self.msg)

        self.cls_pub.publish(self.cls)
        
        if visualize:
            result_img = self.results[0].plot()
            cv2.imshow('result', result_img)
            cv2.waitKey(1)

        self.dt = time.time() - t
        #rospy.loginfo(f" time : {self.dt}")

if __name__=='__main__':
    rospy.init_node("yolohunter")
    yolo = YOLO_cam()

    while not rospy.is_shutdown():
        yolo.post_process(False)