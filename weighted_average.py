#!/usr/bin/env python

"""
    Accepts a heatmap and generates a single point that represents the average
    location of the occupant over a certain period of time. 
"""

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

    def __init__(self, heatmap):
        # constants
        self.map_width = 9.0  # in meters
        self.map_height = 6.0  # in meters
        self.map_resolution = 0.125
        self.number_of_sensors = 0
        self.heatmap = heatmap
        self.result = (0, 0)

    def get_weighted_average_point(self):
        x_average = 0
        y_average = 0

        self.heatmap = self.heatmap.get_normalized_heatmap()

        for x_index in range(len(self.heatmap)):
            for y_index in range(len(self.heatmap[0])):
                if (self.heatmap[x_index][y_index] != 0):
                    x_average += (x_index * self.heatmap[x_index][y_index])
                    y_average += (y_index * self.heatmap[x_index][y_index])

        weighted_average_point = (x_average, y_average)
        self.result = weighted_average_point

        return weighted_average_point

    def display_all(self):
        heatmap_array = self.heatmap.get_heatmap_array()
        np_heatmap = np.array(heatmap_array)
        plt.plot(self.result[0], self.result[1], 'go')

        axis = plt.gca()
        plt.imshow(np.transpose(np_heatmap),
                   cmap='hot', interpolation='nearest')
        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()
