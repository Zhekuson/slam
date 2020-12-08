from landmarks_getter import LandmarksGetter
import os
import b0RemoteApi
import time
from k_junior_robot import KJuniorRobot
from b0RemoteApi import RemoteApiClient
from math import pi
from utils import get_vector_in_robot_coords, get_angle_between_vectors
from extended_kalman_filter import ExtendedKalmanFilter
from image_processor import ImageProcessor


os.add_dll_directory('C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\')


def perform_step(step_func, client: RemoteApiClient):
    client.simxStartSimulation(client.simxServiceCall())
    step_func()
    client.simxStopSimulation(client.simxServiceCall())


with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApi') as client:
    image_processor = ImageProcessor()
    robot = KJuniorRobot(client, 'KJunior', 'Vision_sensor', 'Proximity_sensor0')
    extended_kalman_filter = ExtendedKalmanFilter(robot) 
    landmarks_getter = LandmarksGetter()

    def step():
        robot.subscribe_to_robot_position_change()
        robot.subscribe_to_left_wheel_position_change()
        robot.move(pi / 2, 0)
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

        print(avg_speed / len(times))
        print(avg_angle / len(times))


    def move():
        robot.subscribe_to_robot_position_change()
        robot.subscribe_to_left_wheel_position_change()        
        image_processor.process_image(robot.stop_and_get_image())
        robot.move(pi / 4, 20)
        image_processor.process_image(robot.stop_and_get_image())
        robot.move(pi / 2, 20)
        image_processor.process_image(robot.stop_and_get_image())
        robot.move(pi / 2, 20)
        image_processor.process_image(robot.stop_and_get_image())
        robot.move(pi / 2, 20)
        image_processor.process_image(robot.stop_and_get_image())
        robot.save_trajectory()
        image_processor.wait_remainig_threads()

    
    def test_kalman_filter():
        robot.subscribe_to_robot_position_change()
        robot.subscribe_to_orientation_change()
        extended_kalman_filter.perform_filter_step(landmarks_getter.get_landmarks(image_processor.process_image(robot.stop_and_get_image())))
        robot.move(pi / 3, 0)
        extended_kalman_filter.perform_filter_step(landmarks_getter.get_landmarks(image_processor.process_image(robot.stop_and_get_image())))
        robot.move(pi / 3, 0)
        extended_kalman_filter.perform_filter_step(landmarks_getter.get_landmarks(image_processor.process_image(robot.stop_and_get_image())))
        robot.move(pi / 3, 0)
        extended_kalman_filter.perform_filter_step(landmarks_getter.get_landmarks(image_processor.process_image(robot.stop_and_get_image())))
        robot.move(pi / 3, 0)
        extended_kalman_filter.perform_filter_step(landmarks_getter.get_landmarks(image_processor.process_image(robot.stop_and_get_image())))
        robot.move(pi / 3, 0)
        extended_kalman_filter.perform_filter_step(landmarks_getter.get_landmarks(image_processor.process_image(robot.stop_and_get_image())))
        robot.save_trajectory()
        extended_kalman_filter.plot_positions()


    perform_step(test_kalman_filter, client)