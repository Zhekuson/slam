import os
import b0RemoteApi
import time
from KJuniorRobot import KJuniorRobot
from b0RemoteApi import RemoteApiClient
from math import pi

os.add_dll_directory('C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\')

def perform_step(step_func, client: RemoteApiClient):
    client.simxStartSimulation(client.simxServiceCall())
    step_func()
    #client.simxStopSimulation(client.simxServiceCall())


with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApi') as client:
    robot = KJuniorRobot(client, 'KJunior', 'Vision_sensor5')
    client.simxSynchronous(False)
    def step():
        #robot.subscribe_to_position_change()
        '''
        robot.set_target_speed(10, 5)
        robot.stop()
        robot.set_target_speed(-10, 5)
        robot.stop()
        '''
        robot.rotate_without_moving(pi / 2)
        robot.stop()    

    perform_step(step, client)