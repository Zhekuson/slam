import matplotlib.pyplot as plt
from utils import plot_plot_and_save, clear_folder
import time


class MapBuilder:
    def __init__(self):
        self.depths_estimations_graphics_folder = './images/map_builder/depth_estimations_graphics/'
        self.map_folder = './images/map_builder/maps/'    
        self.map_name = 'map.png'   
        self.cube_line_threshold = 0.008
        self.figure, self.ax = plt.subplots(nrows=1, ncols=1)
        clear_folder(self.depths_estimations_graphics_folder)
        clear_folder(self.map_folder)
        self.updates_count = 0


    def update_map(self, image_parse_data):
        objects_boxes_coords = image_parse_data[0]
        estimated_distances_to_pixels = image_parse_data[1][0][0]
        image_tensor = image_parse_data[2]

        for index, object_box_coords in enumerate(objects_boxes_coords):
            left_up_point = (int(object_box_coords[0] + 1), int(object_box_coords[1] + 1))
            right_bottom_point = (int(object_box_coords[2] - 1), int(object_box_coords[3] - 1))        
            middle_line_index, depths = self._get_cube_configuration(left_up_point, right_bottom_point, estimated_distances_to_pixels)    
        
        self.updates_count += 1


    def _get_cube_configuration(self, left_up_point, right_bottom_point, estimated_distances_to_pixels):
        middle_dot_left = (left_up_point[0], (left_up_point[1] - right_bottom_point[1]) / 2)
        middle_dot_right = (right_bottom_point[0], (left_up_point[1] - right_bottom_point[1]) / 2)  

        depths = []
        index = 0
        min_index = 0

        for y in range(middle_dot_left[0], middle_dot_right[0]):
            depth = estimated_distances_to_pixels[middle_dot_left[0]][y]
            depths.append(depth)

            if (depths[min_index] > depth):
                min_index = index

            index += 1

        curr_time = time.time()
        path = self.depths_estimations_graphics_folder + 'depth_plot_' + str(self.updates_count) + '_' + str(curr_time) + '.png'
        plot_plot_and_save(path, depths)

        delta = (depths[min_index] - depths[0] + depths[min_index] - depths[len(depths) - 1]).item()
        if min_index > 50 and min_index < len(depths) - 50 and delta < 0 and abs(delta) > self.cube_line_threshold:
            return left_up_point[0] + min_index, depths
        
        return -1, depths


    def _get_distances_to_object(self, depths, middle_line_index):
        return [0, 0, 0]
        if middle_line_index == -1:
            avg = sum(depths[100:len(depths) - 100]) / (len(depths) - 200) 
            return [avg, avg, avg]
        else:
            nearest_distance = depths[middle_line_index]
            left_depth = sum(depths[100:150]) / 50
            right_depth = sum(depths[len(depths) - 150:len(depths)-100]) / 50
            return [left_depth, nearest_distance, right_depth]


    def _is_black_pixel(self, image, point):
        return image[0][point[0]][point[1]] + image[1][point[0]][point[1]] + image[2][point[0]][point[1]] == 0