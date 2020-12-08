from k_junior_robot import KJuniorRobot
from utils import clear_folder, plot_scatter_and_save, pi_2_pi
from slam_threading.ParallelExecutor import SlamThread, Job
from matplotlib import pyplot as plt
import time
import numpy as np
import math


Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2 
STATE_SIZE = 3 
LM_SIZE = 2 
M_DIST_TH = 0.00002
DEGREES = 20


class ExtendedKalmanFilter:
    def __init__(self, robot: KJuniorRobot):
        self.robot = robot
        self.path_to_map = './images/extended_kalman_filter/map.png'
        self.is_monitoring = True
        self.fig, self.ax = plt.subplots(nrows=1, ncols=1)
        self.supposed = []
        self.real = []
        self.thread = None
        self.DT = 0.2
        self.PEst = np.eye(STATE_SIZE)
        self.initP = np.eye(2)
        self.xEst = np.zeros((STATE_SIZE, 1))
        self.landmarks = None


    def jacobH(self, q, delta, x, i):
        sq = math.sqrt(q)
        G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]], 
                      [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

        G = G / q
        nLM = self.calc_n_LM()
        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

        F = np.vstack((F1, F2))

        H = G @ F

        return H


    def calc_innovation(self, lm, xEst, PEst, z, LMid):
        delta = lm - xEst[0:2]
        q = (delta.T @ delta)[0, 0]
        zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
        zp = np.array([[math.sqrt(q), pi_2_pi(zangle)]])
        
        y = (z - zp).T
        y[1] = pi_2_pi(y[1])
        
        H = self.jacobH(q, delta, xEst, LMid + 1)
        S = H @ PEst @ H.T + Cx[0:2, 0:2]

        return y, S, H
    
    
    def search_correspond_LM_ID(self, zi):
        nLM = self.calc_n_LM()

        mdist = []

        for i in range(nLM):
            lm = self.get_LM_Pos_from_state(i)
            y, S, H = self.calc_innovation(lm, self.xEst, self.PEst, zi, i)
            mdist.append(y.T @ np.linalg.inv(S) @ y)

        mdist.append(M_DIST_TH)

        minid = mdist.index(min(mdist))

        return minid


    def calc_n_LM(self):
        n = int((len(self.xEst) - STATE_SIZE) / LM_SIZE)
        return n


    def jacob_motion(self, state):
        Fx = np.hstack((np.eye(STATE_SIZE), np.zeros((STATE_SIZE, 0 * LM_SIZE * self.calc_n_LM()))))
        
        v = self.robot.get_supposed_linear_velocity()
        jF = np.array([[0.0, 0.0, -self.DT * v * math.sin(state[2])], [0.0, 0.0, self.DT * v * math.cos(state[2])], [0.0, 0.0, 0.0]])

        G = np.eye(STATE_SIZE) + Fx.T @ jF @ Fx
        return G, Fx


    def ekf_predict(self):
        XY = self.robot.get_supposed_position()
        Theta = self.robot.get_supposed_orientation()
        S = STATE_SIZE
        G, Fx = self.jacob_motion(self.xEst[0:S])

        self.xEst[0] = XY[0]
        self.xEst[1] = XY[1]
        self.xEst[2] = Theta
        self.PEst[0:S, 0:S] = G.T @ self.PEst[0:S, 0:S] @ G + Fx.T @ Cx @ Fx 

    
    def _get_landmarks(self, landmarks):
        def process_landmark(landmark):
            angle = math.pi / 9
            left_x = landmark[0][1].item()
            right_x = landmark[0][2].item()
            depths = landmark[1]
            print(left_x, right_x, len(depths))
            start_angle = angle * left_x / len(depths)
            end_angle = angle * right_x / len(depths)
            delta_angle = (end_angle - start_angle) / 5
            i = 0
            dots = []

            while i < 5:
                dots.append([depths[int(i * len(depths) / 5)] / 10, start_angle + delta_angle * (i + 1)])    
                i += 1

            return dots

        landmarks_dots = []
        for landmark in landmarks:
            for dot in process_landmark(landmark):
                landmarks_dots.append(dot)

        return np.array(landmarks_dots)


    def ekf_update(self, z):
        for iz in range(len(z[:, 0])): 
            minid = self.search_correspond_LM_ID(z[iz, 0:2]) 

            nLM = int((len(self.xEst) - STATE_SIZE) / 2)
            
            if minid == nLM: 
                xAug = np.vstack((self.xEst, self.calc_LM_Pos(z[iz, :])))

                PAug = np.vstack((np.hstack((self.PEst, np.zeros((len(self.xEst), LM_SIZE)))),
                                np.hstack((np.zeros((LM_SIZE, len(self.xEst))), self.initP))))
                self.xEst = xAug
                self.PEst = PAug
 
      
            lm = self.get_LM_Pos_from_state(minid)
    
            y, S, H = self.calc_innovation(lm, self.xEst, self.PEst, z[iz, 0:2], minid)

            K = (self.PEst @ H.T) @ np.linalg.inv(S) # Calculate Kalman Gain
            self.xEst = self.xEst + (K @ y)
            self.PEst = (np.eye(len(self.xEst)) - (K @ H)) @ self.PEst
        
        self.xEst[2] = pi_2_pi(self.xEst[2])
    

    def get_LM_Pos_from_state(self, ind):
        lm = self.xEst[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]
        return lm


    def calc_LM_Pos(self, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = self.xEst[0, 0] + z[0] * math.cos(self.xEst[2, 0] + z[1])
        zp[1, 0] = self.xEst[1, 0] + z[0] * math.sin(self.xEst[2, 0] + z[1])

        return zp

    
    def perform_filter_step(self, landmarks):
        processed_landmarks = self._get_landmarks(landmarks)
        if len(processed_landmarks) > 0:
            self.ekf_predict()
            self.ekf_update(processed_landmarks)
            estimated_position = [self.xEst[0][0], self.xEst[1][0]]
            self.robot.update_supposed_position(estimated_position)
            true_position = self.robot.get_robot_position()
            self.supposed.append(estimated_position)
            self.real.append(true_position)


    def plot_positions(self):
        self.ax.scatter([s[0] for s in self.supposed], [s[1] for s in self.supposed], c = 'g')
        self.ax.scatter([r[0] for r in self.real], [r[1] for r in self.real], c = 'b')
        
        i = 3
        while i + 1 < len(self.xEst):
            print([self.xEst[i]], [self.xEst[i + 1]])
            self.ax.scatter([self.xEst[i]], [self.xEst[i + 1]], c = 'r')
            i += 2

        self.fig.savefig(self.path_to_map)
        plt.close(self.fig)