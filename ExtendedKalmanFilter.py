from KJuniorRobot import KJuniorRobot
from utils import clear_folder, plot_scatter_and_save
from slam_threading.ParallelExecutor import SlamThread, Job
from matplotlib import pyplot as plt
import time


class ExtendedKalmanFilter:
    def __init__(self, robot: KJuniorRobot):
        self.robot = robot
        self.path_to_map = './images/extended_kalman_filter/map.png'
        self.is_monitoring = True
        self.fig, self.ax = plt.subplots(nrows=1, ncols=1)
        self.supposed = []
        self.real = []
        self.thread = None


    def start_monitoring_robot(self):
        def monitor_step():
            supposed = []
            real = []
            while self.is_monitoring:
                true_position = self.robot.get_robot_position()
                supposed_position = self.robot.get_supposed_position()
                orientation = self.robot.get_supposed_orientation()
                supposed.append(supposed_position)
                real.append(true_position)
                time.sleep(0.2)

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
            
