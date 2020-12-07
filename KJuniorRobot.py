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
from utils import get_angle_between_vectors, get_vector_in_robot_coords, clear_folder, plot_scatter_and_save


class KJuniorRobot:

    def __init__(self, client: RemoteApiClient, robot_name, attached_vision_sensor_name,
         attached_proximity_sensor_name):
        """
        Init: already connected client, robot name from Coppelia, vision sensor name
        """
        self.default_left_motor_name = '_motorLeft'
        self.default_right_motor_name = '_motorRight'
        self.client = client

        self.position = [None, None]
        self.trajectory = []
        self.robot_position_lock = Lock()
        self.orientation_lock = Lock()
        self.left_motor_position_lock = Lock()
        self.orientation = []
        self.left_motor_trajectory = []
        self.angular_velocity = 1.4
        self.linear_velocity = 10
        self.rotation_per_sec = 0.12908982839851437
        self.experiments_images_folder = './images/angular_speed_experiments/'
        self.robot_trajectory_images_folder = './images/robot_trajectory/'

        self.supposed_position = [0, 0]
        self.supposed_velocity = 0
        self.supposed_angular_velocity = self.rotation_per_sec
        self.supposed_orientation = 0
        self.suppposed_position_lock = Lock()
        self.supposed_orientation_lock = Lock()

        clear_folder(self.experiments_images_folder)
        clear_folder(self.robot_trajectory_images_folder)

        # robot
        robot_handle = client.simxGetObjectHandle(robot_name, client.simxServiceCall())
        if not robot_handle[0]:
            raise RobotException('Error happened while binding robot (name error?)')

        self.robot_id = robot_handle[1]

        # motors
        left_motor_handle = client.simxGetObjectHandle(robot_name + self.default_left_motor_name, client.simxServiceCall())
        right_motor_handle = client.simxGetObjectHandle(robot_name + self.default_right_motor_name, client.simxServiceCall())

        if not left_motor_handle[0] or not right_motor_handle[0]:
            raise RobotException('Error happened while binding robot (motor name error?)')
        
        self.left_motor_id = left_motor_handle[1]
        self.right_motor_id = right_motor_handle[1]

        # vision
        vision_sensor_handle = client.simxGetObjectHandle(attached_vision_sensor_name, client.simxServiceCall())
        if not vision_sensor_handle[0]:
            raise RobotException('Error happened while binding robot (vision sensor name error?)')

        self.vision_sensor_id = vision_sensor_handle[1]


    def get_target_velocity(self):
        result = self.client.simxGetJointTargetVelocity(self.robot_id, self.client.simxServiceCall())
        if result[0]:
            return result[1]

        raise RobotException("Error in getting target velocity")


    def get_robot_position(self):
        self.robot_position_lock.acquire()
        pos = self.position
        self.robot_position_lock.release()
        return pos


    def save_trajectory(self):
        trajectory = self.get_trajectory()
        x, y = [p[0] for p in trajectory], [p[1] for p in trajectory]
        plot_scatter_and_save(self.robot_trajectory_images_folder + 'robot_trajectory_' + str(time.time()) + '.png', x, y)


    def get_trajectory(self):
        self.robot_position_lock.acquire()
        trajectory = self.trajectory
        self.robot_position_lock.release()
        return trajectory


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

        raise RobotException("Error in getting velocity")


    def get_supposed_position(self):
        self.suppposed_position_lock.acquire()
        pos = self.supposed_position
        self.suppposed_position_lock.release()
        return pos

    
    def get_supposed_orientation(self):
        self.supposed_orientation_lock.acquire()
        orientation = self.supposed_orientation
        self.supposed_orientation_lock.release()
        return orientation


    def get_supposed_linear_velocity(self):
        return self.supposed_velocity


    def get_supposed_angular_velocity(self):
        return self.supposed_angular_velocity


    def get_left_motor_trajectory(self):
        self.left_motor_position_lock.acquire()
        trajectory = [p for p in self.left_motor_trajectory]
        self.left_motor_position_lock.release()
        return trajectory


    def save_left_motor_trajectory(self):
        trajectory = self.get_left_motor_trajectory()
        x, y = [p[0] for p in trajectory], [p[1] for p in trajectory]
        plot_scatter_and_save(self.experiments_images_folder + 'left_motor_trajectory_' + str(time.time()) + '.png', x, y)


    def determine_angular_speed(self, experiment_time, show_trajectory = False):
        x = []
        y = []
        times = []
        angular_velocities = []
        angular_speed_measurements = []
        robot_positions = []
        self.client.simxSetJointTargetVelocity(self.right_motor_id, self.angular_velocity, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.left_motor_id, -self.angular_velocity, self.client.simxDefaultPublisher())

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

        self.stop()

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
            plt.savefig(self.experiments_images_folder + 'exp' + str(experiment_time) + '.png')

        return avg_angular_velocity, rotation_angle


    def move(self, rotation, length):
        print('Rotating on ' + str(rotation) + ' and moving on ' + str(length))
        self._rotate_without_moving(rotation)
        time_for_movement = length / self.linear_velocity
        self._set_target_speed(self.linear_velocity, time_for_movement)


    def _rotate_without_moving(self, delta_angle):
        needed_time_sec = delta_angle / self.rotation_per_sec
        
        self.client.simxSetJointTargetVelocity(self.left_motor_id, -self.angular_velocity, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, self.angular_velocity, self.client.simxDefaultPublisher())
        
        print(needed_time_sec)
        self.supposed_angular_velocity = self.rotation_per_sec

        start_time = time.time()
        last_update_time = start_time
        while time.time() < start_time + needed_time_sec:
            time_delta = time.time() - last_update_time
            last_update_time = time.time()        
            self._update_supposed_orientation(time_delta * self.supposed_angular_velocity)
            self.client.simxSpinOnce()
            
        self.supposed_angular_velocity = 0


    def _update_supposed_orientation(self, da):
        self.supposed_orientation_lock.acquire()
        self.supposed_orientation += da
        self.supposed_orientation_lock.release()


    def _set_target_speed(self, speed, exec_time):
        self.client.simxSetJointTargetVelocity(self.left_motor_id, speed, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, speed, self.client.simxDefaultPublisher())
        self.supposed_velocity = 0.05

        start_time = time.time()
        last_update_time = start_time
        while time.time() < start_time + exec_time:
            time_delta = time.time() - last_update_time
            last_update_time = time.time()
            dy = self.supposed_velocity * math.sin(self.supposed_orientation) * time_delta
            dx = self.supposed_velocity * math.cos(self.supposed_orientation) * time_delta
            self._update_supposed_position(dx, dy)
            self.client.simxSpinOnce()

        self.supposed_velocity = 0
    

    def _update_supposed_position(self, dx, dy):
        self.suppposed_position_lock.acquire()
        self.supposed_position = [self.supposed_position[0] + dx, self.supposed_position[1] + dy]
        self.suppposed_position_lock.release()


    def stop(self):
        self.client.simxSetJointTargetVelocity(self.left_motor_id, 0, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, 0, self.client.simxDefaultPublisher())

        start_time = time.time()
        while time.time() < start_time + 2:
            self.client.simxSpinOnce()


    def stop_and_get_image(self):
        return self.client.simxGetVisionSensorImage(self.vision_sensor_id, False, self.client.simxServiceCall())
