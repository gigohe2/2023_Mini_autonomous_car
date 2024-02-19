#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import matplotlib.pyplot as plt
from icecar.msg import scan_xy
from Util import Carstatus
import math
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped

class Config:
    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = 1

        # Define robot siz
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check



class DWA():
    def __init__(self, loop_hz=10):
        self.rate = rospy.Rate(loop_hz)
        rospy.Subscriber('/filter/quaternion', QuaternionStamped, self.quat_callback)
        self.xy_sub = rospy.Subscriber('scan_xy', scan_xy, self.cb_scan)
        self.obs = None
        self.my_car = Carstatus()
        self.x = [0, 0, 0, 0, 0]
        self.config = Config()
        self.goal = [0, 0] # target point by global path planner
        

    def cb_scan(self, data):
        self.obs = [data.x, data.y]

    def motion(self, x, u, dt):
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x

    def dwa_control(self):
        # x : [x, y, yaw, v, w]
        # config : parameters for algorithm
        # goal : target point published by global path
        # obs : obstacle detected by 2D LiDAR
        # 라이다 기준으로 차량 좌표는 고정임 -> x, y, yaw는 상수 / 속도 및 각속도 필요
        self.x = [0, 0, self.my_car.yaw, self.my_car.v, self.my_car.w]
        
        dw = self.dynamic_window()

        u, trajectory = self.calc_control_and_trajectory(dw)

        return u, trajectory
    
    def dynamic_window(self):
        
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        Vd = [self.x[3] - self.config.max_accel * self.config.dt,
              self.x[3] + self.config.max_accel * self.config.dt,
              self.x[4] - self.config.max_delta_yaw_rate * self.config.dt,
              self.x[4] + self.config.max_delta_yaw_rate * self.config.dt]
        
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        return dw
    
    def calc_control_and_trajectory(self, dw):
        x_init = self.x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([self.x])

        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for y in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                
                trajectory = self.predict_trajectory(x_init, v, y)

                goal_cost = self.config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)
                
                final_cost = goal_cost + speed_cost + ob_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.config.robot_stuck_flag_cons \
                            and abs(self.x[3]) < self.config.robot_stuck_flag_cons:
                        best_u[1] = -self.config.max_delta_yaw_rate
        return best_u, best_trajectory
    
    def calc_to_goal_cost(self, trajectory):
        
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost
    
    def calc_obstacle_cost(self, trajectory):
        ox = self.obs[:, 0]
        oy = self.obs[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = self.obs[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= self.config.robot_length / 2
        right_check = local_ob[:, 1] <= self.config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.config.robot_length / 2
        left_check = local_ob[:, 1] >= -self.config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                        np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    
        min_r = np.min(r)
        return 1.0 / min_r  # OK

    def predict_trajectory(self, x_init, v, y):
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, y], self.config.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.config.dt
        return trajectory



if __name__=='__main__':
    rospy.init_node("lidar")
    lidar = LiDAR_2D()
    plt.figure()
    
    plt.ion()
    plt.show()
    while not rospy.is_shutdown():
        lidar.data_handler()
        lidar.visualize()
        lidar.rate.sleep()