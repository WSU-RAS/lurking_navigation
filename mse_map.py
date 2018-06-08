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
        self.smallest_point = ARBITRARY_LARGE_NUMBER
        self.heatmap = Heatmap()

    def get_heatmap(self, data_filepath):
        self.heatmap.load_from_file(data_filepath)
        self.heatmap = self.heatmap.get_heatmap_array()
        print self.heatmap

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
                mse_map[x_index][y_index] += 1

        return mse_map

    def display_mse_map(self):
        mse_map_array = self.get_mse_map_array()
        np_mse_map = np.array(mse_map_array)
        axis = plt.gca()
        plt.imshow(np_mse_map, cmap='hot', interpolation='nearest')
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