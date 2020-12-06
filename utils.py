import numpy as np
import math
import os, shutil


def get_angle_between_vectors(first, second):
    coords_sum = first[0] * second[0] + first[1] * second[1]
    lengths_product = math.sqrt(first[0] ** 2 + first[1] ** 2) * math.sqrt(second[0] ** 2 + second[1] ** 2)
    cosa = coords_sum / lengths_product
    return np.arccos(cosa)


def get_vector_in_robot_coords(xx, yy, robot_pos):
    return [xx - robot_pos[0], yy - robot_pos[1]]


def clear_folder(folder):
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))