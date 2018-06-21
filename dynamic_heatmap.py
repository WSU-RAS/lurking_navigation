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

HEATMAP_DECAY_STRENGTH = (1 - 0.0231049060187) # 50% every 30 minutes 

class DynamicHeatmap(Heatmap):
    def __init__(self, sensor_list_filepath, config):
        Heatmap.__init__(self, sensor_list_filepath, config)
        self.heatmap = self.get_heatmap_array()
        self.landing_zone = (0,0)

    """
        Called every time a sensor is triggered. 
        If the sensor is in the map, increment it by 1. 
    """
    def update_heatmap(self, triggered_sensor):
        if triggered_sensor in self.sensor_map: 
            sensor_location = self._get_sensor_location(triggered_sensor)
            sensor_location = int(sensor_location[0]/self.map_resolution), int(sensor_location[1]/self.map_resolution)
            self.heatmap[sensor_location[0]][sensor_location[1]] += 1

    def _get_sensor_location(self, triggered_sensor):
        return self.sensor_map[triggered_sensor]

    """
        Decay every point in the heatmap by a certain preset percentage.
        This method is called every TIME_TICK. 
    """
    def decay_heatmap(self):
        for x_index in range(len(self.heatmap)):
            for y_index in range(len(self.heatmap[0])):
                self.heatmap[x_index][y_index] *= HEATMAP_DECAY_STRENGTH # currently 0.8

    """
        Displays heatmap once - called every TIME_TICK from LurkingAI or Simulator
    """
    def display_heatmap(self):
        np_heatmap = np.array(self.heatmap)
        plt.plot(self.landing_zone[0], self.landing_zone[1], 'bo')
        plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')
        axis = plt.gca()
        
        # Checks if our y-axis is aligned with the origin
        if (axis.get_ylim()[0] > axis.get_ylim()[1]):
            axis.set_ylim(axis.get_ylim()[::-1])

        fig = plt.gcf()
        fig.show() 
        fig.canvas.draw() 

    """
        This is included here because the heatmap is displayed in this class. 
    """
    def mark_spot_on_map(self, point):
        self.landing_zone = point

    def get_
