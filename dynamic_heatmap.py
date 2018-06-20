#!/usr/bin/env python
import math
import random
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import rospy

from config import Config
from heatmap import Heatmap
from ras_msgs.msg import SensorPub

TIME_TICK = 1                  # in seconds
HEATMAP_DECAY_STRENGTH = 0.8   # how quickly the heatmap decays

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, config):
        Heatmap.__init__(self, sensor_list_filepath, config)
        self.heatmap = self.get_heatmap_array()

    def update_heatmap(self, triggered_sensor):
        if triggered_sensor in self.sensor_map: 
            sensor_location = self._get_sensor_location(triggered_sensor)
            # x_index = int(point[0] / self.map_resolution)

            sensor_location = int(sensor_location[0]/self.map_resolution), int(sensor_location[1]/self.map_resolution)
            print sensor_location

            self.heatmap[sensor_location[0]][sensor_location[1]] += 1

    def _get_sensor_location(self, triggered_sensor):
        return self.sensor_map[triggered_sensor]

    def _decay_heatmap(self, heatmap):
        """
            Decay every point in the heatmap by a certain preset percentage.
            This method is called every TIME_TICK. 
        """
        for row_index, row in enumerate(heatmap):
            for col_index in enumerate(row): 
                heatmap[row_index][col_index] *= HEATMAP_DECAY_STRENGTH # currently 0.8
        
        return heatmap

    """
        Displays heatmap once - called every time tick from lurking_ai 
    """
    def display_heatmap(self):
        # heatmap_array = self.get_heatmap_array()

        np_heatmap = np.array(self.heatmap)
        plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')

        axis = plt.gca()
        
        if (axis.get_ylim()[0] > axis.get_ylim()[1]):
            axis.set_ylim(axis.get_ylim()[::-1])

        # plt.show
        fig = plt.gcf()
        fig.show() 
        fig.canvas.draw() 