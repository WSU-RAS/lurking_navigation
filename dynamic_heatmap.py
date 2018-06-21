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

HEATMAP_DECAY_STRENGTH = (1 - 0.0231049060187) # how quickly the heatmap decays

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, config):
        Heatmap.__init__(self, sensor_list_filepath, config)
        self.heatmap = self.get_heatmap_array()

    def update_heatmap(self, triggered_sensor):
        if triggered_sensor in self.sensor_map: 
            sensor_location = self._get_sensor_location(triggered_sensor)
            sensor_location = int(sensor_location[0]/self.map_resolution), int(sensor_location[1]/self.map_resolution)
            self.heatmap[sensor_location[0]][sensor_location[1]] += 1

    def _get_sensor_location(self, triggered_sensor):
        return self.sensor_map[triggered_sensor]

    def decay_heatmap(self):
        """
            Decay every point in the heatmap by a certain preset percentage.
            This method is called every TIME_TICK. 
        """
        for row_index in range(len(self.heatmap)):
            for col_index in range(len(self.heatmap[0])):
                self.heatmap[row_index][col_index] *= HEATMAP_DECAY_STRENGTH # currently 0.8

    def display_heatmap(self):
        """
        Displays heatmap once - called every time tick from LurkingAI or Simulator
        """
        np_heatmap = np.array(self.heatmap)
        plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')
        axis = plt.gca()
        
        # Checks if our y-axis is aligned with the origin
        if (axis.get_ylim()[0] > axis.get_ylim()[1]):
            axis.set_ylim(axis.get_ylim()[::-1])

        fig = plt.gcf()
        fig.show() 
        fig.canvas.draw() 
