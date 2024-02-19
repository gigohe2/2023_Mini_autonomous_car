#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from std_msgs.msg import Float32


def callbackFunction(msg):
    global cur_speed
    cur_speed = msg.data
    #print(cur_speed)

def sumMatrix(A, B):
    A = np.array(A)
    B = np.array(B)
    answer = A + B
    return answer.tolist()

def image_callback(msg):
    # try:
    global steer
    cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("Ego Vehicle Camera", cv_image)
    #RGB_camera_in = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    size_im = cv2.resize(cv_image, dsize=(640,480))
    
    roi = size_im[250:400, 100:540]
    roi_im = cv2.resize(roi, (424, 240)) 
    cv2.imshow("ROI", roi_im)

    Blur_im = cv2.bilateralFilter(roi_im, d=-1, sigmaColor=5, sigmaSpace=5)
    edges = cv2.Canny(Blur_im, 50, 100)
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180.0, threshold=25, minLineLength=10, maxLineGap=20)
    pt1_sum_ri = (0, 0)
    pt2_sum_ri = (0, 0)
    pt1_avg_ri = (0, 0)
    count_posi_num_ri = 0

    pt1_sum_le = (0, 0)
    pt2_sum_le = (0, 0)
    pt1_avg_le = (0, 0)

    count_posi_num_le = 0
    if lines is None: #in case HoughLinesP fails to return a set of lines
                #make sure that this is the right shape [[ ]] and ***not*** []
                lines = [[0,0,0,0]]
    else:

        #for line in range(N):
        for line in lines:

            x1, y1, x2, y2 = line[0]

            #x1 = lines[line][0][0]
            #y1 = lines[line][0][1]
            #x2 = lines[line][0][2]
            #y2 = lines[line][0][3]

            if x2 == x1:
                a = 1
            else:
                a = x2 - x1

            b = y2 - y1

            radi = b / a  # 라디안 계산
            # print('radi=', radi)

            theta_atan = math.atan(radi) * 180.0 / math.pi
            # print('theta_atan=', theta_atan)

            pt1_ri = (x1 + 108, y1 + 240)
            pt2_ri = (x2 + 108, y2 + 240)
            pt1_le = (x1 + 108, y1 + 240)
            pt2_le = (x2 + 108, y2 + 240)

            if theta_atan > 20.0 and theta_atan < 90.0:
                # cv2.line(size_im, (x1+108, y1+240), (x2+108, y2+240), (0, 255, 0), 2)
                # print('live_atan=', theta_atan)

                count_posi_num_ri += 1

                pt1_sum_ri = sumMatrix(pt1_ri, pt1_sum_ri)
                # pt1_sum = pt1 + pt1_sum
                # print('pt1_sum=', pt1_sum)

                pt2_sum_ri = sumMatrix(pt2_ri, pt2_sum_ri)
                # pt2_sum = pt2 + pt2_sum
                # print('pt2_sum=', pt2_sum)

            if theta_atan < -20.0 and theta_atan > -90.0:
                # cv2.line(size_im, (x1+108, y1+240), (x2+108, y2+240), (0, 0, 255), 2)
                # print('live_atan=', theta_atan)

                count_posi_num_le += 1

                pt1_sum_le = sumMatrix(pt1_le, pt1_sum_le)
                # pt1_sum = pt1 + pt1_sum
                # print('pt1_sum=', pt1_sum)

                pt2_sum_le = sumMatrix(pt2_le, pt2_sum_le)
                # pt2_sum = pt2 + pt2_sum
                # print('pt2_sum=', pt2_sum)

        # print('pt1_sum=', pt1_sum_ri)
        # print('pt2_sum=', pt2_sum_ri)
        # print('count_posi_num_ri=', count_posi_num_ri)
        # print('count_posi_num_le=', count_posi_num_le)

        # testartu = pt1_sum / np.array(count_posi_num)
        # print(tuple(testartu))

        pt1_avg_ri = pt1_sum_ri // np.array(count_posi_num_ri)
        pt2_avg_ri = pt2_sum_ri // np.array(count_posi_num_ri)
        pt1_avg_le = pt1_sum_le // np.array(count_posi_num_le)
        pt2_avg_le = pt2_sum_le // np.array(count_posi_num_le)

        # print('pt1_avg_ri=', pt1_avg_ri)
        # print('pt2_avg_ri=', pt2_avg_ri)
        # print('pt1_avg_le=', pt1_avg_le)
        # print('pt2_avg_le=', pt2_avg_le)

        # print('pt1_avg=', pt1_avg_ri)
        # print('pt2_avg=', pt2_avg_ri)
        # print('np_count_posi_num=', np.array(count_posi_num))

        # line1_ri = tuple(pt1_avg_ri)
        # line2_ri = tuple(pt2_avg_ri)
        # line1_le = tuple(pt1_avg_le)
        # line2_le = tuple(pt2_avg_le)
        # print('line1=', line1_ri)
        # print('int2=', int2)

        #################################################
        # 차석인식의 흔들림 보정
        # right-----------------------------------------------------------
        x1_avg_ri, y1_avg_ri = pt1_avg_ri
        # print('x1_avg_ri=', x1_avg_ri)
        # print('y1_avg_ri=', y1_avg_ri)
        x2_avg_ri, y2_avg_ri = pt2_avg_ri
        # print('x2_avg_ri=', x2_avg_ri)
        # print('y2_avg_ri=', y2_avg_ri)

        a_avg_ri = ((y2_avg_ri - y1_avg_ri) / (x2_avg_ri - x1_avg_ri))
        b_avg_ri = (y2_avg_ri - (a_avg_ri * x2_avg_ri))
        # print('a_avg_ri=', a_avg_ri)
        # print('b_avg_ri=', b_avg_ri)

        pt2_y2_fi_ri = 480

        # pt2_x2_fi_ri = ((pt2_y2_fi_ri - b_avg_ri) // a_avg_ri)

        if a_avg_ri > 0:
            pt2_x2_fi_ri = int((pt2_y2_fi_ri - b_avg_ri) // a_avg_ri)
        else:
            pt2_x2_fi_ri = 0

        # print('pt2_x2_fi_ri=', pt2_x2_fi_ri)
        pt2_fi_ri = (pt2_x2_fi_ri, pt2_y2_fi_ri)
        # pt2_fi_ri = (int(pt2_x2_fi_ri), pt2_y2_fi_ri)
        # print('pt2_fi_ri=', pt2_fi_ri)

        # left------------------------------------------------------------
        x1_avg_le, y1_avg_le = pt1_avg_le
        x2_avg_le, y2_avg_le = pt2_avg_le
        # print('x1_avg_le=', x1_avg_le)
        # print('y1_avg_le=', y1_avg_le)
        # print('x2_avg_le=', x2_avg_le)
        # print('y2_avg_le=', y2_avg_le)

        a_avg_le = ((y2_avg_le - y1_avg_le) / (x2_avg_le - x1_avg_le))
        b_avg_le = (y2_avg_le - (a_avg_le * x2_avg_le))
        # print('a_avg_le=', a_avg_le)
        # print('b_avg_le=', b_avg_le)

        pt1_y1_fi_le = 480
        if a_avg_le < 0:
            pt1_x1_fi_le = int((pt1_y1_fi_le - b_avg_le) // a_avg_le)
        else:
            pt1_x1_fi_le = 0
        # pt1_x1_fi_le = ((pt1_y1_fi_le - b_avg_le) // a_avg_le)
        # print('pt1_x1_fi_le=', pt1_x1_fi_le)

        pt1_fi_le = (pt1_x1_fi_le, pt1_y1_fi_le)
        # print('pt1_fi_le=', pt1_fi_le)

        # print('pt1_avg_ri=', pt1_sum_ri)
        # print('pt2_fi_ri=', pt2_fi_ri)
        # print('pt1_fi_le=', pt1_fi_le)
        # print('pt2_avg_le=', pt2_sum_le)
        #################################################

        #################################################
        # lane painting
        # right-----------------------------------------------------------
        # cv2.line(size_im, tuple(pt1_avg_ri), tuple(pt2_avg_ri), (0, 255, 0), 2) # right lane
        cv2.line(size_im, tuple(pt1_avg_ri), tuple(pt2_fi_ri), (0, 255, 0), 2)  # right lane
        # left-----------------------------------------------------------
        # cv2.line(size_im, tuple(pt1_avg_le), tuple(pt2_avg_le), (0, 255, 0), 2) # left lane
        cv2.line(size_im, tuple(pt1_fi_le), tuple(pt2_avg_le), (0, 255, 0), 2)  # left lane
        # center-----------------------------------------------------------
        cv2.line(size_im, (320, 480), (320, 360), (0, 228, 255), 1)  # middle lane
        #################################################

        #################################################
        # possible lane
        # FCP = np.array([pt1_avg_ri, pt2_avg_ri, pt1_avg_le, pt2_avg_le])
        # cv2.fillConvexPoly(size_im, FCP, color=(255, 242, 213)) # BGR
        #################################################
        FCP_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8) + 0
        # FCP = np.array([pt1_avg_ri, pt2_avg_ri, pt1_avg_le, pt2_avg_le])
        # FCP = np.array([(100,100), (100,200), (200,200), (200,100)])
        FCP = np.array([pt2_avg_le, pt1_fi_le, pt2_fi_ri, pt1_avg_ri])
        cv2.fillConvexPoly(FCP_img, FCP, color=(255, 242, 213))  # BGR
        alpha = 0.9
        size_im = cv2.addWeighted(size_im, alpha, FCP_img, 1 - alpha, 0)

        # alpha = 0.4
        # size_im = cv2.addWeighted(size_im, alpha, FCP, 1 - alpha, 0)
        #################################################

        #################################################
        # lane center 및 steering 계산 (320, 360)
        lane_center_y_ri = 360
        if a_avg_ri > 0:
            lane_center_x_ri = int((lane_center_y_ri - b_avg_ri) // a_avg_ri)
        else:
            lane_center_x_ri = 0

        lane_center_y_le = 360
        if a_avg_le < 0:
            lane_center_x_le = int((lane_center_y_le - b_avg_le) // a_avg_le)
        else:
            lane_center_x_le = 0

        # caenter left lane (255, 90, 185)
        cv2.line(size_im, (lane_center_x_le, lane_center_y_le - 10), (lane_center_x_le, lane_center_y_le + 10),
                    (0, 228, 255), 1)
        # caenter right lane
        cv2.line(size_im, (lane_center_x_ri, lane_center_y_ri - 10), (lane_center_x_ri, lane_center_y_ri + 10),
                    (0, 228, 255), 1)
        # caenter middle lane
        lane_center_x = ((lane_center_x_ri - lane_center_x_le) // 2) + lane_center_x_le
        cv2.line(size_im, (lane_center_x, lane_center_y_ri - 10), (lane_center_x, lane_center_y_le + 10),
                    (0, 228, 255), 1)

        # print('lane_center_x=', lane_center_x)

        text_left = 'Turn Left'
        text_right = 'Turn Right'
        text_center = 'Center'
        text_non = ''
        org = (320, 440)
        font = cv2.FONT_HERSHEY_SIMPLEX

        if 0 < lane_center_x <= 318:
            steer = -0.1
            cv2.putText(size_im, text_left, org, font, 0.7, (0, 0, 255), 2)
        elif 318 < lane_center_x < 322:
            steer = 0
            # elif lane_center_x > 318 and lane_center_x < 322 :
            cv2.putText(size_im, text_center, org, font, 0.7, (0, 0, 255), 2)
        elif lane_center_x >= 322:
            steer = 0.1
            cv2.putText(size_im, text_right, org, font, 0.7, (0, 0, 255), 2)
        elif lane_center_x == 0:
            steer= 0 
            cv2.putText(size_im, text_non, org, font, 0.7, (0, 0, 255), 2)
        #################################################
        print("lane center : ", lane_center_x)
        global test_con
        test_con = 1
        # print('test_con=', test_con)

        # 변수 초기화
        count_posi_num_ri = 0

        pt1_sum_ri = (0, 0)
        pt2_sum_ri = (0, 0)
        pt1_avg_ri = (0, 0)
        pt2_avg_ri = (0, 0)

        count_posi_num_le = 0

        pt1_sum_le = (0, 0)
        pt2_sum_le = (0, 0)
        pt1_avg_le = (0, 0)
        pt2_avg_le = (0, 0)

        cv2.imshow('frame_size_im', size_im)
        cv2.waitKey(1)
        # cv2.imshow("test_im", test_im) # original size image
        # cv2.waitKey(1)


    #except Exception as e:
        
#PID control
class PID():
    def __init__(self, kp, ki, kd):
        # kp, ki, kd는 PID 제어 상수값
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.p_error=0.0
        self.i_error=0.0
        self.d_error=0.0
        
    def pid_control(self, err):
        # error 계산
        self.p_error = err
        self.i_error += err # Integral, 에러 누적
        self.d_error = err-self.p_error # Differential, 바로 전 error와 차이값 적용
        return (self.kp*self.p_error + self.ki*self.i_error + self.kd*self.d_error)


pid = PID(1, 0, 0)

def main():
    rospy.init_node("image_viewer_node", anonymous=True)
    
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    
    
    while not rospy.is_shutdown():
        continue

if __name__ == "__main__":
    main()