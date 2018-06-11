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

class WeightedAverage():
    """
        Given a heatmap, returns a point on the grid corresponding to the 
        average location of the occupant over a certain period of time 
    """
    def __init__(self):
        # constants
        self.map_width = 9.0 # in meters
        self.map_height = 6.0 # in meters
        self.map_resolution = 0.125
        self.number_of_sensors = 0
        self.heatmap = Heatmap()
        self.point_map = self.heatmap.point_map
        self.map_resolution = self.heatmap.map_resolution

    def get_heatmap(self, data_filepath):
        self.heatmap.load_from_file(data_filepath)
        self.heatmap = self.heatmap.get_heatmap_array()

    def get_weighted_average_point(self):
        x_average = 0
        y_average = 0

        print self.point_map

        for point, value in self.point_map.iteritems():
            x_index = int(point[0] / self.map_resolution)
            y_index = int(point[1] / self.map_resolution)

            x_average += int(x_index * value)
            y_average += int(y_index * value)

        weighted_average_point = (x_average, y_average)
        return weighted_average_point

if __name__ == "__main__":
    # get command line arguments
    data_filepath = sys.argv[1]

    # build the heatmap and mse map
    weighted_average = WeightedAverage() 
    weighted_average.get_heatmap(data_filepath)
    weighted_average_point = weighted_average.get_weighted_average_point() 

    print "The average for the time period provided is", weighted_average_point