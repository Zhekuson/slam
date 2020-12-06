from b0RemoteApi import RemoteApiClient
from RobotException import RobotException
import numpy as np
import time
from threading import Lock
from timer import Timer
from math import pi
import math
from matplotlib import pyplot as plt
import matplotlib.markers
from utils import get_angle_between_vectors, get_vector_in_robot_coords


class KJuniorRobot:

    def __init__(self, client: RemoteApiClient, robot_name, attached_vision_sensor_name,
         attached_proximity_sensor_name):
        """
        Init: already connected client, robot name from Coppelia, vision sensor name
        """
        self.default_left_motor_name = '_motorLeft'
        self.default_right_motor_name = '_motorRight'
        self.client = client
        self.supposedX = 0
        self.supposedY = 0
        self.position = [None, None]
        self.trajectory = []
        self.robot_position_lock = Lock()
        self.orientation_lock = Lock()
        self.left_motor_position_lock = Lock()
        self.orientation = []
        self.left_motor_trajectory = []
        self.angular_velocity = 1
        self.linear_velocity = 10
        self.rotation_per_sec = 0.03011793979849999
        '''
        #proximity sensor
        proximity_handle = client.simxGetObjectHandle(attached_proximity_sensor_name, client.simxServiceCall())
        if(not proximity_handle[0]):
            raise RobotException('error happened while binding robot (proximity sensor error?)')
        self.proximity_id = proximity_handle[1]
        '''
        # robot
        robot_handle = client.simxGetObjectHandle(robot_name, client.simxServiceCall())
        if not robot_handle[0]:
            raise RobotException('error happened while binding robot (name error?)')

        self.robot_id = robot_handle[1]

        # motors
        left_motor_handle = client.simxGetObjectHandle(robot_name + self.default_left_motor_name, client.simxServiceCall())
        right_motor_handle = client.simxGetObjectHandle(robot_name + self.default_right_motor_name, client.simxServiceCall())

        if not left_motor_handle[0] or not right_motor_handle[0]:
            raise RobotException('error happened while binding robot (motor name error?)')
        
        self.left_motor_id = left_motor_handle[1]
        self.right_motor_id = right_motor_handle[1]

        # vision
        vision_sensor_handle = client.simxGetObjectHandle(attached_vision_sensor_name, client.simxServiceCall())
        if not vision_sensor_handle[0]:
            raise RobotException('error happened while binding robot (vision sensor name error?)')

        self.vision_sensor_id = vision_sensor_handle[1]


    def get_target_velocity(self):
        result = self.client.simxGetJointTargetVelocity(self.robot_id, self.client.simxServiceCall())
        if result[0]:
            return result[1]

        raise RobotException("ERROR IN GETTING TARGET VELOCITY")


    def get_robot_position(self):
        self.robot_position_lock.acquire()
        pos = self.position
        self.robot_position_lock.release()
        return pos


    def plot_trajectory(self):
        trajectory = self.get_trajectory()
        plt.scatter([p[0] for p in trajectory], [p[1] for p in trajectory])
        plt.show()


    def get_trajectory(self):
        self.robot_position_lock.acquire()
        trajectory = self.trajectory
        self.robot_position_lock.release()
        return trajectory

    def subscribe_to_proximity_sensor(self):
        def callback(msg):
            print(self.client.simxGetObjectName(msg[4], False,self.client.simxServiceCall()))
        self.client.simxReadProximitySensor(self.proximity_id,self.client.simxDefaultSubscriber(callback))


    def subscribe_to_robot_position_change(self):
        def callback(msg):
            self.robot_position_lock.acquire()
            pos = [msg[1][0], msg[1][1]]
            self.position = pos
            self.trajectory.append(pos)
            self.robot_position_lock.release()

        self.client.simxGetObjectPosition(self.robot_id, -1, self.client.simxDefaultSubscriber(callback))


    def subscribe_to_left_wheel_position_change(self):
        def callback(msg):
            self.left_motor_position_lock.acquire()
            pos = [msg[1][0], msg[1][1]]
            self.left_motor_trajectory.append(pos)
            self.left_motor_position_lock.release()

        self.client.simxGetObjectPosition(self.left_motor_id, -1, self.client.simxDefaultSubscriber(callback))


    def subscribe_to_orientation_change(self):
        def callback(msg):
            self.orientation_lock.acquire()
            self.orientation = msg
            self.orientation_lock.release()

        self.client.simxGetObjectOrientation(self.robot_id, -1, self.client.simxDefaultSubscriber(callback))


    def get_orientation(self):
        self.orientation_lock.acquire()
        orientation = [o for o in self.orientation]
        self.orientation_lock.release()
        return orientation


    def get_current_velocity(self, id):
        result = self.client.simxGetObjectVelocity(id, self.client.simxServiceCall())
        if result[0]:
            return result

        raise RobotException("ERROR IN GETTING CURRENT VELOCITY")


    def get_left_motor_trajectory(self):
        self.left_motor_position_lock.acquire()
        trajectory = [p for p in self.left_motor_trajectory]
        self.left_motor_position_lock.release()
        return trajectory


    def plot_left_motor_trajectory(self):
        trajectory = self.get_left_motor_trajectory()
        plt.scatter([p[0] for p in trajectory], [p[1] for p in trajectory])
        plt.show()


    def determine_angular_speed(self, experiment_time, show_trajectory = False):
        x = []
        y = []
        times = []
        angular_velocities = []
        angular_speed_measurements = []
        robot_positions = []
        self.client.simxSetJointTargetVelocity(self.left_motor_id, -self.angular_velocity, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, self.angular_velocity, self.client.simxDefaultPublisher())

        start_time = time.time()
        while time.time() < start_time + experiment_time:
            self.client.simxSpinOnce()
            pos = self.client.simxGetObjectPosition(self.left_motor_id, -1, self.client.simxServiceCall())
            vel = self.client.simxGetObjectVelocity(self.left_motor_id, self.client.simxServiceCall())
            robot_pos = self.client.simxGetObjectPosition(self.robot_id, -1, self.client.simxServiceCall())
            robot_positions.append([robot_pos[1][0], robot_pos[1][1]])
            angular_speed_measurements.append(vel[2])
            curr_time = time.time()
            times.append(curr_time)
            x.append(pos[1][0])
            y.append(pos[1][1])

        for i in range(1, len(x)):
            dt = times[i] - times[i - 1]
            curr_robot_pos = robot_positions[i - 1]
            first = get_vector_in_robot_coords(x[i], y[i], curr_robot_pos)
            second = get_vector_in_robot_coords(x[i - 1], y[i - 1], curr_robot_pos)
            da = get_angle_between_vectors(first, second)
            angular_velocities.append(da / dt)

        first = get_vector_in_robot_coords(x[0], y[0], robot_positions[0])
        second = get_vector_in_robot_coords(x[len(x) - 1], y[len(y) - 1], robot_positions[len(robot_positions) - 1])
        rotation_angle = get_angle_between_vectors(first, second)

        avg_angular_velocity = sum(angular_velocities) / len(angular_velocities)

        if show_trajectory:
            plt.scatter(x, y)
            marker_sizes = [p for p in range(len(robot_positions))]
            marker_style = matplotlib.markers.MarkerStyle('o', 'none')
            plt.scatter([pos[0] for pos in robot_positions], [pos[1] for pos in robot_positions], s = marker_sizes, marker = marker_style)
            plt.savefig('./images/exp' + str(experiment_time) + '.png')

        return avg_angular_velocity, rotation_angle


    def move(self, rotation, length):
        self.rotate_without_moving(rotation)
        time_for_movement = length / self.linear_velocity
        self.set_target_speed(self.linear_velocity, time_for_movement)


    def _spin(self, exec_time):
        start_time = time.time()
        while time.time() < start_time + exec_time:
            self.client.simxSpinOnce()


    def rotate_without_moving(self, delta_angle):
        needed_time_sec = delta_angle / self.rotation_per_sec
        self.client.simxSetJointTargetVelocity(self.left_motor_id, -self.angular_velocity, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, self.angular_velocity, self.client.simxDefaultPublisher())
        self._spin(needed_time_sec)


    def set_target_speed(self, speed, exec_time):
        print("Setting robot speed to " + str(speed) + " at time " + str(time.time()))
        self.client.simxSetJointTargetVelocity(self.left_motor_id, speed, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, speed, self.client.simxDefaultPublisher())
        self._spin(exec_time)
    

    def stop(self):
        
        self.client.simxSetJointTargetVelocity(self.left_motor_id, 0, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, 0, self.client.simxDefaultPublisher())
        self._spin(3)


    def subscribe_getting_image_from_scanner(self, callback):
        self.client.simxGetVisionSensorImage(self.vision_sensor_id, False, self.client.simxDefaultSubscriber(callback))

