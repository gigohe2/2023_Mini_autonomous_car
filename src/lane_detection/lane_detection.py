#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import cv2 
import numpy as np
import os
import natsort
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

ROI_START_ROW =400 # 차선을 찾을 ROI 영역의 시작 Row값
ROI_END_ROW = 550  # 차선을 찾을 ROT 영역의 끝 Row값
WIDTH, HEIGHT = 1280, 720  # 카메라 이미지 가로x세로 크기
ROI_HEIGHT = ROI_END_ROW - ROI_START_ROW  # ROI 영역의 세로 크기  
L_ROW = int((ROI_END_ROW - ROI_START_ROW)/2)  # 차선의 위치를 찾기 위한 ROI 안에서의 기준 Row값 

class Hough_lane():
    def __init__(self, loop_hz=50, resize=False):
        self.rate = rospy.Rate(loop_hz)
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.cb_cam)
        self.lane_steer_pub = rospy.Publisher('/icelab/lane_steer', Float64, queue_size=1)
        self.img = None
        self.cte = 0
        self.resize = resize
        rospy.wait_for_message('/camera/color/image_raw', Image)


    def cb_cam(self, data):
        if self.resize == True:
            self.img = cv2.resize(self.bridge.imgmsg_to_cv2(data, "bgr8"), (640, 480))
        else:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            



    def grayscale(self, img): 
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    def canny(self, img, low_threshold, high_threshold):
        return cv2.Canny(img, low_threshold, high_threshold)

    def gaussian_blur(self, img, kernel_size): 
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def region_of_interest(self, img, color3=(255,255,255), color1=255): # ROI 셋팅
        vertices = np.array([[(420, ROI_START_ROW),     # 왼쪽 위
                                (860, ROI_START_ROW),    # 오른쪽 위
                                (1180, ROI_END_ROW),       # 오른쪽 아래
                                (100, ROI_END_ROW)]],     # 왼쪽 아래
                                dtype=np.int32)
        mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
        
        if len(img.shape) > 2: # Color 이미지(3채널)라면 :
            color = color3
        else: # 흑백 이미지(1채널)라면 :
            color = color1
            
        # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
        cv2.fillPoly(mask, vertices, color)
        
        # 이미지와 color로 채워진 ROI를 합침
        ROI_image = cv2.bitwise_and(img, mask)
        return ROI_image

    def hough_lines(self,img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
            lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
            self.draw_lines(line_img, lines)
            detected = True
            slopes = []
            filtered_lines = []
            prev_x_left = 0
            prev_x_right = WIDTH

            for line in lines:
                x1, y1, x2, y2 = line[0]

                if (x2 == x1):
                    slope = 1000.0
                else:
                    slope = float(y2-y1) / float(x2-x1)
            
                if 0.2 < abs(slope):
                    slopes.append(slope)
                    filtered_lines.append(line[0])
            if len(filtered_lines) == 0:
                detected = False
            left_lines = []
            right_lines = []

            for j in range(len(slopes)):
                Line = filtered_lines[j]
                slope = slopes[j]

                x1,y1, x2,y2 = Line

                # 기울기 값이 음수이고 화면의 왼쪽에 있으면 왼쪽 차선으로 분류함
                # 기준이 되는 X좌표값 = (화면중심값 - Margin값)
                Margin = 0
                
                if (slope < 0) and (x2 < WIDTH/2-Margin):
                    left_lines.append(Line.tolist())

                # 기울기 값이 양수이고 화면의 오른쪽에 있으면 오른쪽 차선으로 분류함
                # 기준이 되는 X좌표값 = (화면중심값 + Margin값)
                elif (slope > 0) and (x1 > WIDTH/2+Margin):
                    right_lines.append(Line.tolist())
            # 왼쪽 차선을 표시하는 대표직선을 구함        
            m_left, b_left = 0.0, 0.0
            x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

            # 왼쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
            size = len(left_lines)
            if size != 0:
                for line in left_lines:
                    x1, y1, x2, y2 = line
                    x_sum += x1 + x2
                    y_sum += y1 + y2
                    if(x2 != x1):
                        m_sum += float(y2-y1)/float(x2-x1)
                    else:
                        m_sum += 0                
                    
                x_avg = x_sum / (size*2)
                y_avg = y_sum / (size*2)
                m_left = m_sum / size
                b_left = y_avg - m_left * x_avg

                if m_left != 0.0:
                    #=========================================
                    # (직선 #1) y = mx + b 
                    # (직선 #2) y = 0
                    # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
                    x1 = int((0.0 - b_left) / m_left)

                    #=========================================
                    # (직선 #1) y = mx + b 
                    # (직선 #2) y = ROI_HEIGHT
                    # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
                    x2 = int((ROI_HEIGHT - b_left) / m_left)

                    

            # 오른쪽 차선을 표시하는 대표직선을 구함      
            m_right, b_right = 0.0, 0.0
            x_sum, y_sum, m_sum = 0.0, 0.0, 0.0

            # 오른쪽 차선을 표시하는 선분들의 기울기와 양끝점들의 평균값을 찾아 대표직선을 구함
            size = len(right_lines)
            if size != 0:
                for line in right_lines:
                    x1, y1, x2, y2 = line
                    x_sum += x1 + x2
                    y_sum += y1 + y2
                    if(x2 != x1):
                        m_sum += float(y2-y1)/float(x2-x1)
                    else:
                        m_sum += 0     
            
                x_avg = x_sum / (size*2)
                y_avg = y_sum / (size*2)
                m_right = m_sum / size
                b_right = y_avg - m_right * x_avg

                if m_right != 0.0:
                    #=========================================
                    # (직선 #1) y = mx + b 
                    # (직선 #2) y = 0
                    # 위 두 직선의 교점의 좌표값 (x1, 0)을 구함.           
                    x1 = int((0.0 - b_right) / m_right)

                    #=========================================
                    # (직선 #1) y = mx + b 
                    # (직선 #2) y = ROI_HEIGHT
                    # 위 두 직선의 교점의 좌표값 (x2, ROI_HEIGHT)을 구함.               
                    x2 = int((ROI_HEIGHT - b_right) / m_right)

                

            #=========================================
            # 차선의 위치를 찾기 위한 기준선(수평선)은 아래와 같음.
            #   (직선의 방정식) y = L_ROW 
            # 위에서 구한 2개의 대표직선, 
            #   (직선의 방정식) y = (m_left)x + (b_left)
            #   (직선의 방정식) y = (m_right)x + (b_right)
            # 기준선(수평선)과 대표직선과의 교점인 x_left와 x_right를 찾음.
            #=========================================

            #=========================================        
            # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
            # 이 경우에는 교점 좌표값을 기존 저장해 놨던 값으로 세팅함 
            #=========================================
            if m_left == 0.0:
                x_left = prev_x_left  # 변수에 저장해 놓았던 이전 값을 가져옴

            #=========================================
            # 아래 2개 직선의 교점을 구함
            # (직선의 방정식) y = L_ROW  
            # (직선의 방정식) y = (m_left)x + (b_left)
            #=========================================
            else:
                
                x_left = int((L_ROW - b_left) / m_left)
                                
            #=========================================
            # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
            # 이 경우에는 교점 좌표값을 기존 저장해 놨던 값으로 세팅함 
            #=========================================
            if m_right == 0.0:
                x_right = prev_x_right  # 변수에 저장해 놓았던 이전 값을 가져옴	
            
            #=========================================
            # 아래 2개 직선의 교점을 구함
            # (직선의 방정식) y = L_ROW  
            # (직선의 방정식) y = (m_right)x + (b_right)
            #=========================================
            else:
                x_right = int((L_ROW - b_right) / m_right)
            
            #=========================================
            # 대표직선의 기울기 값이 0.0이라는 것은 직선을 찾지 못했다는 의미임
            # 이 경우에 반대쪽 차선의 위치 정보를 이용해서 내 위치값을 정함 
            #=========================================
            if m_left == 0.0 and m_right != 0.0:
                x_left = x_right - 380

            if m_left != 0.0 and m_right == 0.0:
                x_right = x_left + 380

            # 이번에 구한 값으로 예전 값을 업데이트 함			
            prev_x_left = x_left
            prev_x_right = x_right
            
            # 왼쪽 차선의 위치와 오른쪽 차선의 위치의 중간 위치를 구함
            x_midpoint = (x_left + x_right) // 2 


            return line_img, detected, x_left, x_right, x_midpoint

    def draw_lines(self, img, lines, color=[0, 0, 255], thickness=2): # 선 그리기
        if lines is not None:
            for line in lines:
                for x1,y1,x2,y2 in line:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)
        cv2.line(img, (0, ROI_START_ROW+L_ROW), (WIDTH, ROI_START_ROW+L_ROW), [255, 255, 255], thickness=10)
    def weighted_img(self, img, initial_img, α=1, β=1., λ=0.): # 두 이미지 operlap 하기
        return cv2.addWeighted(initial_img, α, img, β, λ)
    
    def main(self):
        p1 =  [400, 400]  # 좌상
        p2 =  [880, 400] # 좌하
        p3 =  [1280, 550] # 우상
        p4 = [0, 550]  # 우하

        # corners_point_arr는 변환 이전 이미지 좌표 4개 
        corner_points_arr = np.float32([p1, p2, p3, p4])
        height, width = self.img.shape[:2]


        image_p1 = [0, 0]
        image_p2 = [width, 0]
        image_p3 = [width, height]
        image_p4 = [0, height]

        image_params = np.float32([image_p1, image_p2, image_p3, image_p4])


        mat = cv2.getPerspectiveTransform(corner_points_arr, image_params)
        image_transformed = cv2.warpPerspective(self.img, mat, (width, height))

        # cv2.imshow('bev', image_transformed)
        # cv2.waitKey(1)

        src = cv2.cvtColor(image_transformed, cv2.COLOR_RGB2GRAY)
        t, t_otsu = cv2.threshold(src, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        hough_img, detected, x_left, x_right, x_midpoint = self.hough_lines(t_otsu, 1, 1*np.pi/180, 30, 10, 20)
        error = (x_midpoint - 1280/2) /2
        
        steer = np.interp(error, [-640, 640], [0, 1]) 
        
        #print(x_left, x_right, x_midpoint, detected, error, steer)
        self.lane_steer_pub.publish(steer)
        


if __name__ == "__main__":
    rospy.init_node('lanedetection')
    lane_detector = Hough_lane()

    while not rospy.is_shutdown():
        lane_detector.main()
        lane_detector.rate.sleep()
    