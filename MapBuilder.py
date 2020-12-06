import matplotlib.pyplot as plt


class MapBuilder:
    def __init__(self):
        self.map_folder = './images/map_builder/'    
        self.map_name = 'map.png'   
        self.cube_line_threshold = 0.008
        self.figure, self.ax = plt.subplots(nrows=1, ncols=1)


    def update_map(self, image_parse_data):
        pass
        # objects_boxes_coords = image_parse_data[0]
        # estimated_distances_to_pixels = image_parse_data[1][0][0]
        # image_tensor = image_parse_data[2]
        # print(image_tensor.size())

        # for index, object_box_coords in enumerate(objects_boxes_coords):
        #     print('Box ' + str(index))
        #     left_up_point = (int(object_box_coords[0] + 1), int(object_box_coords[1] + 1))
        #     right_bottom_point = (int(object_box_coords[2] - 1), int(object_box_coords[3] - 1))        
        #     middle_line_index, depths = self._get_cube_configuration(left_up_point, right_bottom_point, estimated_distances_to_pixels)    
        #     print(self._get_distance_to_object(middle_line_index, depths))


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

        plt.plot(depths)
        plt.show()

        delta = (depths[min_index] - depths[0] + depths[min_index] - depths[len(depths) - 1]).item()
        if min_index > 50 and min_index < len(depths) - 50 and delta < 0 and abs(delta) > self.cube_line_threshold:
            return left_up_point[0] + min_index, depths
        
        return -1, depths


    def _get_distance_to_object(self, depths, middle_line_index):
        print(depths)
        if middle_line_index == -1:
            avg = sum(depths[100:len(depths) - 100]) / (len(depths) - 200) 
            return avg


    def _plot_object_on_map(object_coords):
        self.ax.scatter()

    def _is_black_pixel(self, image, point):
        return image[0][point[0]][point[1]] + image[1][point[0]][point[1]] + image[2][point[0]][point[1]] == 0