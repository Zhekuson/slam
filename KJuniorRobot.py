from b0RemoteApi import RemoteApiClient
from RobotException import RobotException
import numpy as np
import time
from threading import Lock
from timer import Timer
from math import pi

class KJuniorRobot:

    def __init__(self, client: RemoteApiClient, robot_name, attached_vision_sensor_name):
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
        self.lock = Lock()
        self.orientation_lock = Lock()
        self.orientation = []
        self.w = 4
        self.rotation_per_sec = 0.125

        # robot
        robot_handle = client.simxGetObjectHandle(robot_name, client.simxServiceCall())
        if not robot_handle[0]:
            raise RobotException('error happened while binding robot (name error?)')

        self.robot_id = robot_handle[1]

        # motors
        left_motor_handle = client.simxGetObjectHandle(robot_name +
                                                       self.default_left_motor_name, client.simxServiceCall())
        right_motor_handle = client.simxGetObjectHandle(robot_name +
                                                        self.default_right_motor_name, client.simxServiceCall())
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


    def get_position(self):
        self.lock.acquire()
        pos = self.position
        self.lock.release()
        return pos


    def subscribe_to_position_change(self):
        def callback(msg):
            self.lock.acquire()
            self.position = msg
            self.lock.release()

        self.client.simxGetObjectPosition(self.robot_id, -1, self.client.simxDefaultSubscriber(callback))


    def subscribe_to_orientation_change(self):
        def callback(msg):
            self.orientation_lock.acquire()
            self.orientation = msg
            self.orientation_lock.release()

        self.client.simxGetObjectOrientation(self.robot_id, -1, self.client.simxDefaultSubscriber(callback))


    def get_orientation(self):
        self.orientation_lock.acquire()
        orientation = self.orientation
        self.orientation_lock.release()
        return orientation


    def get_current_velocity(self):
        result = self.client.simxGetObjectVelocity(self.left_motor_id, self.client.simxServiceCall())
        if result[0]:
            return result

        raise RobotException("ERROR IN GETTING CURRENT VELOCITY")

    
    def collect_robot_coords(self):
        def timer_func():
            pos = self.get_position()
            self.trajectory.append(pos)
        
        timer = Timer(timer_func, 1)
        timer.start()

    def rotate_without_moving(self, delta_angle):
        """
        w_speed - rotating speed (depending on sign)
        """

        needed_time_sec = delta_angle / self.rotation_per_sec
        print(needed_time_sec)
        self.client.simxSetJointTargetVelocity(self.left_motor_id, -self.w, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, self.w, self.client.simxDefaultPublisher())
        start_time = time.time()
        while time.time() < start_time + needed_time_sec:
            self.client.simxSpinOnce()
            print("Angular + " + str(self.get_current_velocity()[2]))
            print("Linear + " + str(self.get_current_velocity()[1]))



    def set_target_speed(self, speed, exec_time):
        """
        set speed, bot is moving only forward
        """
        print("Setting roboto speed to " + str(speed) + " at time " + str(time.time()))
        self.client.simxSetJointTargetVelocity(self.left_motor_id, speed, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, speed, self.client.simxDefaultPublisher())
    
        start_time = time.time()
        while time.time() < start_time + exec_time:
            self.client.simxSpinOnce()
    

    def stop(self):
        self.client.simxSetJointTargetVelocity(self.left_motor_id, 0, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.right_motor_id, 0, self.client.simxDefaultPublisher())

        start_time = time.time()
        while time.time() < start_time + 2:
            self.client.simxSpinOnce()
        print("Stopping robot at time " + str(time.time()))


    def get_image_from_scanner(self):
        """
        returns RGB image
        """
        result = self.client.simxGetVisionSensorImage(self.vision_sensor_id,
                                                      False, self.client.simxServiceCall())
        if not result[0]:
            raise RobotException("ERROR IN GETTING IMAGE FROM SCANNER")
        input = np.asarray(bytearray(result[2]), dtype=np.uint8)
        image = input.reshape(result[1][0], result[1][1], 3)
        return image
