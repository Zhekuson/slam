import matplotlib.pyplot as plt
from utils import plot_plot_and_save, clear_folder
import time


class LandmarksGetter:
    def __init__(self):
        self.depths_estimations_graphics_folder = './images/map_builder/depth_estimations_graphics/'
        self.map_folder = './images/map_builder/maps/'    
        self.map_name = 'map.png'   
        self.cube_line_threshold = 0.008
        self.figure, self.ax = plt.subplots(nrows=1, ncols=1)
        clear_folder(self.depths_estimations_graphics_folder)
        clear_folder(self.map_folder)
        self.updates_count = 0

    def get_landmarks(self, image_parse_data):
        objects_boxes_coords = image_parse_data[0]
        estimated_distances_to_pixels = image_parse_data[1][0][0]
        
        objects_boxes_and_depths = []
        for object_box_coords in objects_boxes_coords:
            left_up_point = (int(object_box_coords[0]), int(object_box_coords[1]))
            right_bottom_point = (int(object_box_coords[2]), int(object_box_coords[3]))        
            depths = self._get_cube_configuration(left_up_point, right_bottom_point, estimated_distances_to_pixels)
            objects_boxes_and_depths.append((object_box_coords, depths))

        self.updates_count += 1
        return objects_boxes_and_depths
        

    def _get_cube_configuration(self, left_up_point, right_bottom_point, estimated_distances_to_pixels):
        middle_dot_left = (left_up_point[0], (left_up_point[1] - right_bottom_point[1]) / 2)
        middle_dot_right = (right_bottom_point[0], (left_up_point[1] - right_bottom_point[1]) / 2)  

        depths = []
        index = 0
        min_index = 0

        for y in range(middle_dot_left[0], middle_dot_right[0]):
            depth = estimated_distances_to_pixels[middle_dot_left[0]][y]
            depths.append(depth.item())

            if (depths[min_index] > depth):
                min_index = index

            index += 1

        curr_time = time.time()
        path = self.depths_estimations_graphics_folder + 'depth_plot_' + str(self.updates_count) + '_' + str(curr_time) + '.png'
        plot_plot_and_save(path, depths)

        return depths