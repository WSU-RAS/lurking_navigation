# constructs an MSE map based on the heatmap generated from the data 

# language imports
import sys
import string
import math

# library imports
import matplotlib.pyplot as plt
import numpy as np

# Constants
ARBITRARY_LARGE_NUMBER = 9999

from heatmap import Heatmap

class MSEmap():
    """
        Given a heatmap, constructs an MSE map and returns the "ideal" 
        grid spot containing the minimum value. 
    """
    def __init__(self):
        # constants
        self.map_width = 9.0 # in meters
        self.map_height = 6.0 # in meters
        self.map_resolution = 0.125
        self.number_of_sensors = 0
        self.smallest_point = ARBITRARY_LARGE_NUMBER
        self.heatmap = Heatmap()
        self.point_map = self.heatmap.point_map
        self.map_resolution = self.heatmap.map_resolution

    def get_heatmap(self, data_filepath):
        self.heatmap.load_from_file(data_filepath)
        self.heatmap = self.heatmap.get_heatmap_array()

    def get_mse_map_array(self):
        """ 
            get mse_map array
        """
        mse_map = [
                    [0.0 for j in range(int(self.map_height / self.map_resolution))] 
                    for i in range(int(self.map_width / self.map_resolution))
                  ]

        # load things into the mse_map
        for x_index in range(len(mse_map)):
            for y_index in range(len(mse_map[0])): 
                mse_map[x_index][y_index] = self.calculate_mse_at_point(x_index, y_index)

        return mse_map

    def calculate_mse_at_point(self, x_index, y_index):
        mse_at_point = 0

        for sensor, value in self.point_map:
            distance_to_sensor = self.get_distance_to_sensor(sensor, x_index, y_index)
            weight_of_sensor = self.get_weight_of_sensor(sensor)
            print weight_of_sensor

            mse_at_point += (pow(distance_to_sensor, 2) * weight_of_sensor) 

        return mse_at_point

    def get_distance_to_sensor(self, sensor, grid_x_index, grid_y_index):
        # get (x,y) of sensor 
        sensor_x_index = int(sensor[0] / self.map_resolution)
        sensor_y_index = int(sensor[1] / self.map_resolution)

        # find difference
        x_difference = abs(sensor_x_index - grid_x_index)
        y_difference = abs(sensor_y_index - grid_y_index)

        distance = math.sqrt(pow(x_difference, 2) + pow(y_difference, 2))

        return distance

    def get_weight_of_sensor(self, sensor):
        return self.point_map[sensor]

    def display_mse_map(self):
        mse_map_array = self.get_mse_map_array()
        np_mse_map = np.array(mse_map_array)
        axis = plt.gca()
        plt.imshow(np_mse_map, cmap='hot', interpolation=None)
        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()

if __name__ == "__main__":
    # get command line arguments
    data_filepath = sys.argv[1]

    # build the heatmap and mse map
    mse_map = MSEmap() 
    mse_map.get_heatmap(data_filepath)

    # display the mse map 
    mse_map.display_mse_map()