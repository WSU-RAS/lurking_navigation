# constructs an MSE map based on the heatmap generated from the data 

# language imports
import sys
import string

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
        self.map_width = 6.0 # in meters
        self.map_height = 9.0 # in meters
        self.map_resolution = 0.125
        self.number_of_sensors = 0
        self.smallest_point = ARBITRARY_LARGE_NUMBER
        self.heatmap = Heatmap()
        self.sensors_list = [[]]

    def get_heatmap(self, data_filepath):
        self.heatmap.load_from_file(data_filepath)
        self.heatmap = self.heatmap.get_heatmap_array()

    def get_sensors_from_heatmap(self):
        # convert to a numpy array so we can use numpy tools 
        heatmap_array = np.array(self.heatmap)

        for x_index in range(heatmap_array.shape[0]):
            for y_index in range(heatmap_array.shape[1]): 
                if heatmap_array[x_index][y_index] != 0:
                    self.number_of_sensors += 1
                    #self.sensors_list[][] = heatmap_array[x_index][y_index]
                    #self.sensors_list[][] = 

                    print self.sensors_list
        return 42

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
            for y_index in range(len(mse_map)): 
                mse_map[x_index][y_index] = self.calculate_mse_at_point()

        return mse_map

    def calculate_mse_at_point(self):
        # for every sensor in the sensor array: (pull from heatmap)
        # add to the total:
        # (get_distance)^2 * (weight of sensor)
        return 42

    def get_distance_to_sensor(self, mse_map_point, sensor_name):
        # convert gridpoint to meters
        # get (x,y) of sensor 
        # find difference
        return 42

    def get_weight_of_sensor(self, sensor_name):
        return 42

    def display_mse_map(self):
        mse_map_array = self.get_mse_map_array()
        np_mse_map = np.array(mse_map_array)
        axis = plt.gca()
        plt.imshow(np_mse_map, cmap='hot', interpolation='nearest')
        axis.set_ylim(axis.get_ylim()[::-1])
        # plt.show()

if __name__ == "__main__":
    # get command line arguments
    data_filepath = sys.argv[1]

    # build the heatmap and mse map
    mse_map = MSEmap() 
    mse_map.get_heatmap(data_filepath)
    mse_map.get_sensors_from_heatmap()

    # display the mse map 
    mse_map.display_mse_map()