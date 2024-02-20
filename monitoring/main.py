#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import cv2
import rospy

from std_msgs.msg import Float64    # voltage, 
from geometry_msgs.msg import Vector3Stamped # location
from xsens_mti_driver.msg import XsStatusWord   #status
from sensor_msgs.msg import Image   #camera
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32, Float32MultiArray

MODES = {
    0: "DRIVE_MANUAL",
    1: "DRIVE_GPS",
    2: "DRIVE_ACC",
    3: "DRIVE_LANE"
}

CLASSES = {
    0: "Hunter",
    1: "Person"
}

#UI파일 연결
#단, UI파일은 Python 코드 파일과 같은 디렉토리에 위치해야한다.
form_class = uic.loadUiType("/home/icelab_nx/icecar_monitoring/main.ui")[0]

class RosThred(QThread):
    #초기화 메서드 구현    
    update_image_1 = pyqtSignal(QImage)
    update_info_1 = pyqtSignal(dict)
    
    update_image_2 = pyqtSignal(QImage)
    update_info_2 = pyqtSignal(dict)
    
    def __init__(self, parent): #parent는 WndowClass에서 전달하는 self이다.(WidnowClass의 인스턴스)
        super().__init__(parent)    
        self.parent = parent #self.parent를 사용하여 WindowClass 위젯을 제어할 수 있다.

        self.bridge = CvBridge()
        
        # 차량 상태 정보
        self.parent.v_status = {'speed':0,'location':[0,0],'voltage':0,'rtk':11}
        self.parent.Img_info = {"Mode":0, "Distance":0, "Object detect":0, "Lane detect":0}
        
        self.speed_sub = rospy.Subscriber('/filter/velocity',Vector3Stamped, self.speed_callback)
        self.location_sub = rospy.Subscriber('/filter/positionlla',Vector3Stamped,self.location_callback)
        self.voltage_sub = rospy.Subscriber('/voltage',Float64, self.voltage_callback)
        self.rtk_sub = rospy.Subscriber('/status',XsStatusWord,self.rtk_callback)

        self.mode = None
        self.dist = None
        self.cls = None
        self.lane = None
        rospy.Subscriber('/icelab/yolo/result', Image, self.cb_result)
        rospy.Subscriber('/icelab/driving_mode', Int32, self.cb_mode)
        rospy.Subscriber('/icelab/yolo/classes', Float32MultiArray, self.cb_classes)
        rospy.Subscriber('/icelab/is_lane', Int32, self.cb_lane)
        rospy.Subscriber('/icelab/yolo/hunter_info', Float32MultiArray, self.cb_hunter)
        rospy.wait_for_message('/icelab/yolo/result', Image)
        rospy.wait_for_message('/icelab/driving_mode', Int32)
        rospy.wait_for_message('/icelab/yolo/classes', Float32MultiArray)
        rospy.wait_for_message('/icelab/is_lane', Int32)
        

    #쓰레드로 동작시킬 함수
    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_info_1.emit(self.parent.v_status)
            self.update_info_2.emit(self.parent.Img_info)
            
            rospy.loginfo(f"speed:{self.parent.v_status['speed']}, location:{self.parent.v_status['location']}, voltage:{self.parent.v_status['voltage']}, RTK_state:{self.parent.v_status['rtk']}")
            rate.sleep()

    # 차량 정보 callback
    def speed_callback(self,msg):
        self.parent.v_status['speed'] = msg.vector.x
    
    def voltage_callback(self,msg):
        self.parent.v_status['voltage'] = msg.data

    def location_callback(self,msg):
        temp_xy = msg.vector        
        self.parent.v_status['location'] = [temp_xy.x, temp_xy.y]
        
    def rtk_callback(self,msg):
        self.parent.v_status['rtk'] = msg.rtk_status
        
        
    # 이미지 정보 callback
    def cb_result(self, msg):
        self.parent.RGB_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # OpenCV 이미지를 QImage로 변환합니다.
        height, width, channel = self.parent.RGB_img.shape
        bytes_per_line = 3 * width
        q_img = QImage(self.parent.RGB_img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

        # 이미지 업데이트 신호를 발생시킵니다.
        self.update_image_2.emit(q_img)

    def cb_mode(self, msg):
        self.parent.Img_info["Mode"] = msg.data
        # self.mode = msg.data
    
    def cb_hunter(self, msg):
        self.parent.Img_info["Distance"] = msg.data[0]
        # self.dist = msg.data[0]
        
    def cb_classes(self, msg):
        self.parent.Img_info["Object detect"] = msg.data
        # self.cls = msg.data
        
    def cb_lane(self, msg):
        self.parent.Img_info["Lane detect"] = msg.data
        # self.lane = msg.data


#화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, form_class) :
    def __init__(self) :
        super().__init__()
        self.setupUi(self)
        
        self.v_status = None
        ros = RosThred(self)
        ros.update_info_1.connect(self.update_state_table)
        ros.update_info_2.connect(self.create_table)
        ros.update_image_2.connect(self.update_image)
        ros.start()
        self.update_state_table()
    

    def update_image(self, q_img):
        # QLabel에 표시할 QPixmap 객체를 생성합니다.
        pixmap = QPixmap.fromImage(q_img)

        # QLabel의 setPixmap() 메서드를 사용하여 이미지를 설정합니다.
        # self.label_img는 UI에서 이미지를 표시할 QLabel의 객체 이름이어야 합니다.
        self.camera.setPixmap(pixmap.scaled(self.camera.width(), self.camera.height(), Qt.KeepAspectRatio))

    def update_state_table(self):
        self.vehicle_state_information.setRowCount(4)
        self.vehicle_state_information.setColumnCount(1)
        
        h_header = self.vehicle_state_information.horizontalHeader()
        h_header.setStretchLastSection(True)
        h_header.setSectionResizeMode(QHeaderView.Stretch)
        
        v_header = self.vehicle_state_information.verticalHeader()
        v_header.setStretchLastSection(True)
        v_header.setSectionResizeMode(QHeaderView.Stretch)
        
        self.vehicle_state_information.setItem(0,0,QTableWidgetItem(str(self.v_status['speed'])+" m/s"))
        self.vehicle_state_information.setItem(1,0,QTableWidgetItem(str(self.v_status['location'])))
        self.vehicle_state_information.setItem(2,0,QTableWidgetItem(str(self.v_status['voltage'])+' V'))
        self.vehicle_state_information.setItem(3,0,QTableWidgetItem(str(self.v_status['rtk'])))
        
    def create_table(self):                       
        self.image_information.setRowCount(4)
        self.image_information.setColumnCount(1)
        h_header = self.image_information.horizontalHeader()
        h_header.setStretchLastSection(True)
        h_header.setSectionResizeMode(QHeaderView.Stretch)
        
        v_header = self.image_information.verticalHeader()
        v_header.setStretchLastSection(True)
        v_header.setSectionResizeMode(QHeaderView.Stretch)
        
        item_1 = QTableWidgetItem(str(MODES[self.Img_info["Mode"]]))
        item_2 = QTableWidgetItem(str(self.Img_info["Distance"]) + "m")
        detected_classes = self.Img_info["Object detect"]
        class_names = [CLASSES.get(cls_id, "Unknown") for cls_id in detected_classes]
        detected_string = ", ".join(class_names)
        item_4 = QTableWidgetItem(str(bool(self.Img_info["Lane detect"])))

        self.image_information.setItem(0,0,item_1)
        self.image_information.setItem(1,0,item_2)
        self.image_information.setItem(2, 0, QTableWidgetItem(detected_string))
        self.image_information.setItem(3,0,item_4)
        
        

if __name__ == "__main__" :
    rospy.init_node('icecar_monitor')

    #QApplication : 프로그램을 실행시켜주는 클래스
    app = QApplication(sys.argv) 

    #WindowClass의 인스턴스 생성
    myWindow = WindowClass() 

    #프로그램 화면을 보여주는 코드
    myWindow.show()

    #프로그램을 이벤트루프로 진입시키는(프로그램을 작동시키는) 코드
    app.exec_()