#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import numpy as np
from numpy.linalg import inv

class Kalman():
    def __init__(self, A, H, P0, x0, Q, R, dt=0.02):
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P0
        self.dt = dt
        self.x_esti = x0
    
    def update(self, z_meas, dt=-1):
        if dt == -1:
            dt = self.dt

        x_pred = np.matmul(self.A, self.x_esti)
        P_pred = np.matmul(np.matmul(self.A, self.P), self.A.T) + self.Q
        K = np.matmul(P_pred,np.matmul(self.H.T, inv(np.matmul(np.matmul(self.H,P_pred), self.H.T) + self.R)))
        self.x_esti = x_pred + np.matmul(K, (z_meas - np.matmul(self.H, x_pred)))
        self.P = P_pred - np.matmul(K, np.matmul(self.H, P_pred))

        return self.x_esti, self.P

class KalmanDist():
    def __init__(self):
        self.P = np.array([[6]])
        self.A = np.array([[1]])
        self.H = np.array([[1]])
        self.Q = np.array([[0]])
        self.R = np.array([[4]])
        self.x = np.array([[0]])


    def update(self, z):
        xp = self.A.dot(self.x)
        Pp = self.A.dot(self.P).dot(self.A.T) + self.Q
        K = Pp.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
        x = xp + K.dot(z - self.H.dot(xp))
        self.P = Pp - K.dot(self.H).dot(Pp)
        return x

class KalmanPos2Vel():
    def __init__(self, P0, x0, q1=1, q2=100, r=1000, dt=0.01):
        """칼만 필터 클래스 초기화 함수"""
        self.A = np.array([[1, dt],
                           [0, 1]])
        self.H = np.array([[1, 0]])
        self.Q = np.array([[q1, 0],
                           [0, q2]])
        self.R = np.array([[r]])
        self.P = P0
        self.dt = dt
        self.x_esti = x0

    def update(self, z_meas, dt):
        self.A[0,1] = dt
        x_pred = np.matmul(self.A, self.x_esti)
        P_pred = np.matmul(np.matmul(self.A, self.P), self.A.T) + self.Q
        K = np.matmul(P_pred,np.matmul(self.H.T, inv(np.matmul(np.matmul(self.H,P_pred), self.H.T) + self.R)))
        self.x_esti = x_pred + np.matmul(K, (z_meas - np.matmul(self.H, x_pred)))
        self.P = P_pred - np.matmul(K, np.matmul(self.H, P_pred))

        return self.x_esti[1][0]*3.6, self.P
    

class KalmanRelativeVelocity:
    def __init__(self, initial_velocity, process_noise, measurement_noise):
        self.state = np.array([[initial_velocity], [0]])  
        self.P = np.eye(2)
        self.Q = np.array([[process_noise, 0], [0, process_noise]])
        self.R = measurement_noise
        self.A = np.array([[1, 1], [0, 1]])
        self.H = np.array([[1, 0]])
    
    
    def update(self, measurement, dt):
        self.A[0,1] = dt
        self.state = np.dot(self.A, self.state)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(self.H @ self.P @ self.H.T + self.R))
        self.state = self.state + np.dot(K, (measurement - np.dot(self.H, self.state)))
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

        return self.state[0, 0]
    
