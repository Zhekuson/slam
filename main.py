from MapBuilder import MapBuilder
import os
import b0RemoteApi
import time
from KJuniorRobot import KJuniorRobot
from b0RemoteApi import RemoteApiClient
from math import pi
from utils import get_vector_in_robot_coords, get_angle_between_vectors
from ImageProcessor import ImageProcessor
os.add_dll_directory('C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\')

def perform_step(step_func, client: RemoteApiClient):
    client.simxStartSimulation(client.simxServiceCall())

    try:
        step_func()
    except Exception as ex:
        print(ex)
    
    client.simxStopSimulation(client.simxServiceCall())


with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApi') as client:
    map_builder = MapBuilder()
    image_processor = ImageProcessor(map_builder.update_map)
    robot = KJuniorRobot(client, 'KJunior', 'Vision_sensor5')
    client.simxSynchronous(False)
    def step():
        robot.subscribe_to_robot_position_change()
        robot.subscribe_to_left_wheel_position_change()
        robot.rotate_without_moving(pi / 2)
        robot.stop()

        trajectory = robot.get_left_motor_trajectory()
        robot_pos = robot.get_robot_position()
        first = get_vector_in_robot_coords(trajectory[0][0], trajectory[0][1], robot_pos)
        second = get_vector_in_robot_coords(trajectory[len(trajectory) - 1][0], trajectory[len(trajectory) - 1][1], robot_pos)
        print(get_angle_between_vectors(first, second) - pi / 2)

    def experiment_with_angular_speed():
        avg_speed = 0
        times = [10 for _ in range(20)]
        avg_angle = 0
        for i in range(len(times)):
            w, angle = robot.determine_angular_speed(times[i], True)
            avg_speed += w
            avg_angle += angle
            print(w, angle)
            robot.stop()

        print(avg_speed / len(times))
        print(avg_angle / len(times))

    def move():
        robot.subscribe_to_robot_position_change()
        robot.subscribe_to_left_wheel_position_change()
        robot.subscribe_getting_image_from_scanner(image_processor.subscribe)
        robot.rotate_without_moving(pi / 4)
        robot.set_target_speed(5, 8)
        robot.rotate_without_moving(pi / 2)
        robot.set_target_speed(5, 8)
        robot.rotate_without_moving(pi / 2)
        robot.set_target_speed(5, 8)
        robot.rotate_without_moving(pi / 2)
        robot.set_target_speed(5, 8)
        robot.plot_trajectory()


    perform_step(move, client)