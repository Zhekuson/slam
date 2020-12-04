
from b0RemoteApi import RemoteApiClient
import numpy as np

class KJuniorRobot():
    def print_error_exit(self, msg:str):
        print('''
            ************************************************
            '''+ msg +'''
            ************************************************
            ''')
        exit()
    def __init__(self, client:RemoteApiClient, robot_name, attached_vision_sensor_name):
        '''
        Init: already connected client, robot name from Coppelia, vision sensor name 
        '''
        #init defaults
        self.default_left_motor_name = '_motorLeft'
        self.default_right_motor_name = '_motorRight'
        self.client = client
        self.supposedX = 0
        self.supposedY = 0
        self.default_speed = 10
        self.default_w_speed = 10
        #robot
        robot_handle = client.simxGetObjectHandle(robot_name,client.simxServiceCall())
        if(not robot_handle[0]):
            self.print_error_exit('error happened while binding robot (name error?)')
        
        self.robot_id = robot_handle[1]
        #motors
        left_motor_handle = client.simxGetObjectHandle(robot_name + 
                                self.default_left_motor_name, client.simxServiceCall())
        right_motor_handle = client.simxGetObjectHandle(robot_name + 
                                self.default_right_motor_name, client.simxServiceCall())
        if (not left_motor_handle[0] or not right_motor_handle[0]):
            self.print_error_exit('error happened while binding robot (motor name error?)')
        self.left_motor_id = left_motor_handle[1]
        self.right_motor_id = right_motor_handle[1]
        #vision
        vision_sensor_handle = client.simxGetObjectHandle(attached_vision_sensor_name,client.simxServiceCall())
        if(not vision_sensor_handle[0]):
            self.print_error_exit('error happened while binding robot (vision sensor name error?)')
        self.vision_sensor_id = vision_sensor_handle[1]

    def getTargetVelocity(self):
        result = self.client.simxGetJointTargetVelocity(self.robot_id, self.client.simxServiceCall())
        if(result[0]):
            return result[1]
        self.print_error_exit("ERROR IN GETTING TARGET VELOCITY")

    def getRealCurrentPosition(self):
        result = self.client.simxGetJointPosition(self.robot_id, self.client.simxServiceCall())
        if(result[0]):
            return result[1]
        self.print_error_exit("ERROR IN GETTING CURRENT POSITION")

    def getCurrentVelocity(self):
        result = self.client.simxGetObjectVelocity(self.robot_id, self.client.simxServiceCall())
        if(result[0]):
            return result[1]
        self.print_error_exit("ERROR IN GETTING CURRENT VELOCITY")
    
    def setRotationSpeed(self,w_speed):
        '''
        w_speed - rotating speed (depending on sign)
        '''
        self.client.simxSetJointTargetVelocity(self.left_motor_id,w_speed,
                self.client.simxServiceCall())
        self.client.simxSetJointTargetVelocity(self.right_motor_id,-w_speed,
                self.client.simxServiceCall())

    def setSpeed(self, speed):
        '''
        set speed, bot is moving only forward 
        '''
        self.client.simxSetJointTargetVelocity(self.left_motor_id,speed,
                self.client.simxServiceCall())
        self.client.simxSetJointTargetVelocity(self.right_motor_id,speed,
                self.client.simxServiceCall())
        
    def getImageFromScanner(self):
        '''
        returns RGB image 
        '''
        result = self.client.simxGetVisionSensorImage(self.vision_sensor_id, 
            False, self.client.simxServiceCall())
        if(not result[0]):
            self.print_error_exit("ERROR IN GETTING IMAGE FROM SCANNER")
        input = np.asarray(bytearray(result[2]), dtype=np.uint8)
        image = input.reshape(result[1][0],result[1][1], 3)
        return image
    
    
         


        