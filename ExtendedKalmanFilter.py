from KJuniorRobot import KJuniorRobot
from utils import clear_folder, plot_scatter_and_save
from slam_threading.ParallelExecutor import SlamThread, Job
from matplotlib import pyplot as plt
import time
import numpy as np
import math
# EKF state covariance
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2 # Change in covariance
STATE_SIZE = 3 # x y theta 
LM_SIZE = 2 # x y
M_DIST_TH = 0.2
DEGREES = 20



def get_observations_z():
    return np.array([[18.50309239, 2.57207212,  0.        ],
 [ 9.54827122, -2.98028081,  1.        ],
 [ 5.00819595,  1.15005942,  2.        ],
 [13.30839334,  0.60162481, 3.        ]])

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


    def jacobH(self, q, delta, x, i):
        """
        Calculates the jacobian of the measurement function
        
        :param q:     the range from the system pose to the landmark
        :param delta: the difference between a landmark position and the estimated system position
        :param x:     the state, including the estimated system position
        :param i:     landmark id + 1
        :returns:     the jacobian H
        """
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
        """
        Calculates the innovation based on expected position and landmark position
        
        :param lm:   landmark position
        :param xEst: estimated position/state
        :param PEst: estimated covariance
        :param z:    read measurements
        :param LMid: landmark id
        :returns:    returns the innovation y, and the jacobian H, and S, used to calculate the Kalman Gain
        """
        def pi_2_pi(angle):
            return (angle + math.pi) % (2 * math.pi) - math.pi
        delta = lm - xEst[0:2]
        q = (delta.T @ delta)[0, 0]
        zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
        zp = np.array([[math.sqrt(q), pi_2_pi(zangle)]])
        # zp is the expected measurement based on xEst and the expected landmark position
        
        y = (z - zp).T # y = innovation
        y[1] = pi_2_pi(y[1])
        
        H = self.jacobH(q, delta, xEst, LMid + 1)
        S = H @ PEst @ H.T + Cx[0:2, 0:2]

        return y, S, H
    
    
    def search_correspond_LM_ID(self, zi):
        """
        Landmark association with Mahalanobis distance.
        
        If this landmark is at least M_DIST_TH units away from all known landmarks, 
        it is a NEW landmark.
        
        :param xAug: The estimated state
        :param PAug: The estimated covariance
        :param zi:   the read measurements of specific landmark
        :returns:    landmark id
        """

        nLM = self.calc_n_LM()

        mdist = []

        for i in range(nLM):
            lm = self.get_LM_Pos_from_state(i)
            y, S, H = self.calc_innovation(lm, self.xEst, self.PEst, zi, i)
            mdist.append(y.T @ np.linalg.inv(S) @ y)

        mdist.append(M_DIST_TH)  # new landmark

        minid = mdist.index(min(mdist))

        return minid


    def calc_n_LM(self):
        """
        Calculates the number of landmarks currently tracked in the state
        :param x: the state
        :returns: the number of landmarks n
        """
        n = int((len(self.xEst) - STATE_SIZE) / LM_SIZE)
        return n


    def jacob_motion(self, state):
        """
        Calculates the jacobian of motion model. 
        
        :param x: The state, including the estimated position of the system
        :param u: The control function
        :returns: G:  Jacobian
                Fx: STATE_SIZE x (STATE_SIZE + 2 * num_landmarks) matrix where the left side is an identity matrix
        """
    
        # [eye(3) [0 x y; 0 x y; 0 x y]]
        Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
            (STATE_SIZE, 0*LM_SIZE * self.calc_n_LM()))))# todo 
        
        v = self.robot.get_supposed_linear_velocity()
        jF = np.array([[0.0, 0.0, -self.DT * v * math.sin(state[2])],
                    [0.0, 0.0, self.DT * v * math.cos(state[2])],
                    [0.0, 0.0, 0.0]])
        G = np.eye(STATE_SIZE) + Fx.T @ jF @ Fx
        if self.calc_n_LM() > 0:
            print(Fx.shape)
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

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def ekf_update(self, z):
        """
        Performs the update step of EKF SLAM
            
            :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
            :param PEst:  nxn the predicted covariance
            :param u:     2x1 the control function 
            :param z:     the measurements read at new position
            :param initP: 2x2 an identity matrix acting as the initial covariance
            :returns:     the updated state and covariance for the system
            """
        for iz in range(len(z[:, 0])):  # for each observation
            minid = self.search_correspond_LM_ID(z[iz, 0:2]) # associate to a known landmark

            nLM = int((len(self.xEst) - STATE_SIZE) / 2) # number of landmarks we currently know about
            
            if minid == nLM: # Landmark is a NEW landmark
                print("New LM")
                # Extend state and covariance matrix
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
        
        self.xEst[2] = self.pi_2_pi(self.xEst[2])
    
    def get_LM_Pos_from_state(self, ind):
        """
        Returns the position of a given landmark
        
        :param x:   The state containing all landmark positions
        :param ind: landmark id
        :returns:   The position of the landmark
        """
        lm = self.xEst[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]
        return lm


    def calc_LM_Pos(self, z):
        """
        Calcualtes the pose in the world coordinate frame of a landmark at the given measurement. 

        :param x: [x; y; theta]
        :param z: [range; bearing]
        :returns: [x; y] for given measurement
        """
        zp = np.zeros((2, 1))

        zp[0, 0] = self.xEst[0, 0] + z[0] * math.cos(self.xEst[2, 0] + z[1])
        zp[1, 0] = self.xEst[1, 0] + z[0] * math.sin(self.xEst[2, 0] + z[1])

        return zp

    
    def start_monitoring_robot(self):
        def monitor_step():
            supposed = []
            real = []
            while self.is_monitoring:

                self.ekf_predict()
                self.ekf_update(get_observations_z())


                true_position = self.robot.get_robot_position()
                supposed_position = self.robot.get_supposed_position()
                orientation = self.robot.get_supposed_orientation()
                supposed.append(supposed_position)
                real.append(true_position)
                time.sleep(self.DT)

            return supposed, real 

        self.thread = SlamThread(Job(monitor_step, "Supposed position paint"), True)
        self.thread.start()


    def stop(self):
        self.is_monitoring = False
        while not self.thread.done:
            pass

        supposed, real = self.thread.result
        self.ax.scatter([s[0] for s in supposed], [s[1] for s in supposed])
        self.ax.scatter([r[0] for r in real], [r[1] for r in real])
        self.fig.savefig(self.path_to_map)
        plt.close(self.fig)
            
