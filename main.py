import os
os.add_dll_directory('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu')


import b0RemoteApi
import math
import time
import KJuniorRobot

with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:
    